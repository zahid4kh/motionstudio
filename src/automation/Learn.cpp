#include "Learn.h"
#include <QJsonArray>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <random>

namespace learn {

namespace {

QString packFloats(const std::vector<float>& v) {
    const QByteArray raw(reinterpret_cast<const char*>(v.data()), qsizetype(v.size() * sizeof(float)));
    return QString::fromLatin1(qCompress(raw).toBase64());
}

bool unpackFloats(const QString& s, size_t n, std::vector<float>* out) {
    const QByteArray raw = qUncompress(QByteArray::fromBase64(s.toLatin1()));
    if (raw.size() != qsizetype(n * sizeof(float))) return false;
    out->resize(n);
    std::memcpy(out->data(), raw.constData(), size_t(raw.size()));
    for (float f : *out)
        if (!std::isfinite(f)) return false;
    return true;
}

double wrap(double a) {
    a = std::fmod(a + M_PI, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a - M_PI;
}

void standardize(const std::vector<float>& D, int n, int cols, std::vector<float>* mean, std::vector<float>* sd) {
    mean->assign(size_t(cols), 0.f);
    sd->assign(size_t(cols), 0.f);
    std::vector<double> m(size_t(cols), 0.0), q(size_t(cols), 0.0);
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < cols; ++c) m[size_t(c)] += D[size_t(r) * size_t(cols) + size_t(c)];
    for (int c = 0; c < cols; ++c) m[size_t(c)] /= std::max(1, n);
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < cols; ++c) {
            const double d = D[size_t(r) * size_t(cols) + size_t(c)] - m[size_t(c)];
            q[size_t(c)] += d * d;
        }
    for (int c = 0; c < cols; ++c) {
        (*mean)[size_t(c)] = float(m[size_t(c)]);
        const double s = std::sqrt(q[size_t(c)] / std::max(1, n));
        (*sd)[size_t(c)] = s < 1e-6 ? 1.f : float(s);
    }
}

} // namespace

// ── Mlp ──────────────────────────────────────────────────────────────────────

Mlp::Mlp(int nIn, int nOut, const std::vector<int>& hidden, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<int> sizes{nIn};
    sizes.insert(sizes.end(), hidden.begin(), hidden.end());
    sizes.push_back(nOut);
    for (size_t k = 0; k + 1 < sizes.size(); ++k) {
        Layer L;
        L.in = sizes[k];
        L.out = sizes[k + 1];
        std::normal_distribution<float> init(0.f, float(std::sqrt(1.0 / L.in)));
        L.W.resize(size_t(L.in) * size_t(L.out));
        for (float& w : L.W) w = init(rng);
        L.b.assign(size_t(L.out), 0.f);
        m_layers.push_back(std::move(L));
    }
    m_xMean.assign(size_t(nIn), 0.f);
    m_xStd.assign(size_t(nIn), 1.f);
    m_yMean.assign(size_t(nOut), 0.f);
    m_yStd.assign(size_t(nOut), 1.f);
}

void Mlp::predict(const float* x, float* y) const {
    if (m_layers.empty()) return;
    std::vector<float> h(static_cast<size_t>(nIn())), next;
    for (int i = 0; i < nIn(); ++i) h[size_t(i)] = (x[i] - m_xMean[size_t(i)]) / m_xStd[size_t(i)];
    for (size_t k = 0; k < m_layers.size(); ++k) {
        const Layer& L = m_layers[k];
        next = L.b;
        for (int i = 0; i < L.in; ++i) {
            const float a = h[size_t(i)];
            const float* w = &L.W[size_t(i) * size_t(L.out)];
            for (int j = 0; j < L.out; ++j) next[size_t(j)] += a * w[j];
        }
        if (k + 1 < m_layers.size())
            for (float& v : next) v = std::tanh(v);
        h.swap(next);
    }
    for (int j = 0; j < nOut(); ++j) y[j] = h[size_t(j)] * m_yStd[size_t(j)] + m_yMean[size_t(j)];
}

bool Mlp::fit(const std::vector<float>& X, const std::vector<float>& Y, int n, const Fit& o) {
    const int nI = nIn(), nO = nOut();
    if (n <= 0 || m_layers.empty()) return true;
    standardize(X, n, nI, &m_xMean, &m_xStd);
    standardize(Y, n, nO, &m_yMean, &m_yStd);
    std::vector<float> Xs(X.size()), Ys(Y.size());
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < nI; ++c) {
            const size_t k = size_t(r) * size_t(nI) + size_t(c);
            Xs[k] = (X[k] - m_xMean[size_t(c)]) / m_xStd[size_t(c)];
        }
        for (int c = 0; c < nO; ++c) {
            const size_t k = size_t(r) * size_t(nO) + size_t(c);
            Ys[k] = (Y[k] - m_yMean[size_t(c)]) / m_yStd[size_t(c)];
        }
    }

    const size_t L = m_layers.size();
    const int B = std::max(1, o.batch);
    // Adam state, gradients, activations per layer
    std::vector<std::vector<float>> mW(L), vW(L), mB(L), vB(L), gW(L), gB(L);
    for (size_t k = 0; k < L; ++k) {
        mW[k].assign(m_layers[k].W.size(), 0.f);
        vW[k] = mW[k];
        gW[k] = mW[k];
        mB[k].assign(m_layers[k].b.size(), 0.f);
        vB[k] = mB[k];
        gB[k] = mB[k];
    }
    std::vector<std::vector<float>> acts(L + 1);
    std::vector<float> g, gPrev;
    std::vector<int> order(static_cast<size_t>(n));
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng(o.seed);
    const double b1 = 0.9, b2 = 0.999, eps = 1e-8;
    long step = 0;

    for (int epoch = 0; epoch < o.epochs; ++epoch) {
        const double rate = o.lr * (0.1 + 0.9 * 0.5 * (1 + std::cos(M_PI * epoch / std::max(1, o.epochs))));
        std::shuffle(order.begin(), order.end(), rng);
        double total = 0;
        for (int s = 0; s < n; s += B) {
            const int m = std::min(B, n - s);
            // Forward
            acts[0].resize(size_t(m) * size_t(nI));
            for (int r = 0; r < m; ++r)
                std::copy_n(&Xs[size_t(order[size_t(s + r)]) * size_t(nI)], nI, &acts[0][size_t(r) * size_t(nI)]);
            for (size_t k = 0; k < L; ++k) {
                const Layer& Ly = m_layers[k];
                std::vector<float>& in = acts[k];
                std::vector<float>& out = acts[k + 1];
                out.resize(size_t(m) * size_t(Ly.out));
                for (int r = 0; r < m; ++r) {
                    float* o2 = &out[size_t(r) * size_t(Ly.out)];
                    std::copy(Ly.b.begin(), Ly.b.end(), o2);
                    const float* a = &in[size_t(r) * size_t(Ly.in)];
                    for (int i = 0; i < Ly.in; ++i) {
                        const float ai = a[i];
                        const float* w = &Ly.W[size_t(i) * size_t(Ly.out)];
                        for (int j = 0; j < Ly.out; ++j) o2[j] += ai * w[j];
                    }
                    if (k + 1 < L)
                        for (int j = 0; j < Ly.out; ++j) o2[j] = std::tanh(o2[j]);
                }
            }
            // Loss and its gradient
            g.resize(size_t(m) * size_t(nO));
            const std::vector<float>& out = acts[L];
            for (int r = 0; r < m; ++r) {
                const float* y = &Ys[size_t(order[size_t(s + r)]) * size_t(nO)];
                for (int j = 0; j < nO; ++j) {
                    const float e = out[size_t(r) * size_t(nO) + size_t(j)] - y[j];
                    total += double(e) * e;
                    g[size_t(r) * size_t(nO) + size_t(j)] = 2.f * e / float(m);
                }
            }
            // Backward
            for (size_t kk = L; kk-- > 0;) {
                const Layer& Ly = m_layers[kk];
                const std::vector<float>& a = acts[kk];
                std::fill(gW[kk].begin(), gW[kk].end(), 0.f);
                std::fill(gB[kk].begin(), gB[kk].end(), 0.f);
                for (int r = 0; r < m; ++r) {
                    const float* gr = &g[size_t(r) * size_t(Ly.out)];
                    const float* ar = &a[size_t(r) * size_t(Ly.in)];
                    for (int j = 0; j < Ly.out; ++j) gB[kk][size_t(j)] += gr[j];
                    for (int i = 0; i < Ly.in; ++i) {
                        const float ai = ar[i];
                        float* gw = &gW[kk][size_t(i) * size_t(Ly.out)];
                        for (int j = 0; j < Ly.out; ++j) gw[j] += ai * gr[j];
                    }
                }
                if (kk > 0) {
                    gPrev.assign(size_t(m) * size_t(Ly.in), 0.f);
                    for (int r = 0; r < m; ++r) {
                        const float* gr = &g[size_t(r) * size_t(Ly.out)];
                        const float* ar = &a[size_t(r) * size_t(Ly.in)];
                        float* gp = &gPrev[size_t(r) * size_t(Ly.in)];
                        for (int i = 0; i < Ly.in; ++i) {
                            const float* w = &Ly.W[size_t(i) * size_t(Ly.out)];
                            float d = 0;
                            for (int j = 0; j < Ly.out; ++j) d += gr[j] * w[j];
                            gp[i] = d * (1.f - ar[i] * ar[i]);
                        }
                    }
                    g.swap(gPrev);
                }
            }
            // Adam
            ++step;
            const double c1 = 1 - std::pow(b1, double(step)), c2 = 1 - std::pow(b2, double(step));
            auto adam = [&](std::vector<float>& p, const std::vector<float>& gr, std::vector<float>& mm, std::vector<float>& vv) {
                for (size_t i = 0; i < p.size(); ++i) {
                    mm[i] = float(b1 * mm[i] + (1 - b1) * gr[i]);
                    vv[i] = float(b2 * vv[i] + (1 - b2) * double(gr[i]) * gr[i]);
                    p[i] -= float(rate * (mm[i] / c1) / (std::sqrt(vv[i] / c2) + eps));
                }
            };
            for (size_t k = 0; k < L; ++k) {
                adam(m_layers[k].W, gW[k], mW[k], vW[k]);
                adam(m_layers[k].b, gB[k], mB[k], vB[k]);
            }
        }
        m_loss = total / (double(n) * nO);
        if (o.progress && !o.progress(epoch, m_loss)) return false;
    }
    return true;
}

QJsonObject Mlp::toJson() const {
    QJsonArray sizes;
    std::vector<float> all;
    for (const Layer& L : m_layers) sizes.append(L.in);
    if (!m_layers.empty()) sizes.append(m_layers.back().out);
    auto add = [&](const std::vector<float>& v) { all.insert(all.end(), v.begin(), v.end()); };
    add(m_xMean); add(m_xStd); add(m_yMean); add(m_yStd);
    for (const Layer& L : m_layers) { add(L.W); add(L.b); }
    return QJsonObject{{"sizes", sizes}, {"values", int(all.size())}, {"data", packFloats(all)}, {"loss", m_loss}};
}

bool Mlp::fromJson(const QJsonObject& o, Mlp* out) {
    std::vector<int> sizes;
    for (const QJsonValue& v : o["sizes"].toArray()) {
        const int s = v.toInt();
        if (s <= 0 || s > 100000) return false;
        sizes.push_back(s);
    }
    if (sizes.size() < 2) return false;
    size_t n = size_t(2 * sizes.front() + 2 * sizes.back());
    for (size_t k = 0; k + 1 < sizes.size(); ++k) n += size_t(sizes[k]) * size_t(sizes[k + 1]) + size_t(sizes[k + 1]);
    if (size_t(o["values"].toInt()) != n) return false;
    std::vector<float> all;
    if (!unpackFloats(o["data"].toString(), n, &all)) return false;
    Mlp m;
    size_t at = 0;
    auto take = [&](size_t count) {
        std::vector<float> v(all.begin() + long(at), all.begin() + long(at + count));
        at += count;
        return v;
    };
    m.m_xMean = take(size_t(sizes.front()));
    m.m_xStd  = take(size_t(sizes.front()));
    m.m_yMean = take(size_t(sizes.back()));
    m.m_yStd  = take(size_t(sizes.back()));
    for (size_t k = 0; k + 1 < sizes.size(); ++k) {
        Layer L;
        L.in = sizes[k];
        L.out = sizes[k + 1];
        L.W = take(size_t(L.in) * size_t(L.out));
        L.b = take(size_t(L.out));
        m.m_layers.push_back(std::move(L));
    }
    m.m_loss = o["loss"].toDouble();
    *out = std::move(m);
    return true;
}

// ── AnimationModel ───────────────────────────────────────────────────────────

void AnimationModel::inputs(const std::vector<double>& w, double tau, float* x) const {
    int k = 0;
    for (double v : w) x[k++] = float(v);
    for (int h = 1; h <= harmonics; ++h) x[k++] = float(std::sin(tau * h * M_PI));
    for (int h = 1; h <= harmonics; ++h) x[k++] = float(std::cos(tau * h * M_PI));
    x[k] = float(tau);
}

bool AnimationModel::train(const std::vector<std::pair<QString, take::Take>>& takes, const Options& opt,
                           AnimationModel* out, QString* error, const std::function<bool(int, int, double)>& progress)
{
    auto fail = [&](const QString& why) {
        if (error) *error = why;
        return false;
    };
    if (takes.empty()) return fail("no motions to learn");
    AnimationModel m;
    m.harmonics = std::max(1, opt.harmonics);
    m.joints = takes.front().second.jointColumns();
    m.robot = takes.front().second.robot;
    if (m.joints.isEmpty()) return fail(QString("%1 has no joint columns").arg(takes.front().first));
    m.place = true;
    for (const auto& [name, t] : takes) {
        if (t.rows.size() < 2) return fail(QString("%1 is too short (fewer than two samples)").arg(name));
        QStringList missing;
        for (const QString& j : m.joints)
            if (t.column(j) < 0) missing << j;
        if (!missing.isEmpty())
            return fail(QString("%1 lacks the joints %2: learn motions of one robot together")
                            .arg(name, missing.mid(0, 8).join(", ")));
        if (t.column("x") < 0 || t.column("z") < 0) m.place = false;
        m.clips << name;
        m.durations.push_back(std::max(1e-6, t.duration() - t.rows.front()[0]));
    }
    const int J = int(m.joints.size()), C = int(m.clips.size());
    const int nIn = C + 2 * m.harmonics + 1, nOut = J + (m.place ? 4 : 0);
    std::vector<float> X, Y;
    int n = 0;
    for (int c = 0; c < C; ++c) {
        const take::Take& t = takes[size_t(c)].second;
        std::vector<double> w(size_t(C), 0.0);
        w[size_t(c)] = 1;
        std::vector<int> cols;
        for (const QString& j : m.joints) cols.push_back(t.column(j));
        const int cx = t.column("x"), cy = t.column("y"), cz = t.column("z");
        const int ch = t.column("heading") >= 0 ? t.column("heading") : t.column("ry");
        const std::vector<double>& first = t.rows.front();
        const double t0 = first[0], span = std::max(1e-9, t.rows.back()[0] - t0);
        const double h0 = ch >= 0 ? qDegreesToRadians(first[size_t(ch)]) : 0.0;
        double turned = 0, prevH = h0;
        std::vector<float> x(static_cast<size_t>(nIn));
        for (const std::vector<double>& row : t.rows) {
            m.inputs(w, (row[0] - t0) / span, x.data());
            X.insert(X.end(), x.begin(), x.end());
            for (int col : cols) Y.push_back(float(row[size_t(col)]));
            if (m.place) {
                const double dx = row[size_t(cx)] - first[size_t(cx)], dz = row[size_t(cz)] - first[size_t(cz)];
                const double c0 = std::cos(h0), s0 = std::sin(h0);
                const double h = ch >= 0 ? qDegreesToRadians(row[size_t(ch)]) : h0;
                turned += wrap(h - prevH);   // unwrapped: a clip may turn round more than once
                prevH = h;
                Y.push_back(float(dx * c0 - dz * s0));                  // ahead (mm)
                Y.push_back(float(-dx * s0 - dz * c0));                 // to its left (mm)
                Y.push_back(float(qRadiansToDegrees(turned)));          // turn (degrees)
                Y.push_back(float(cy >= 0 ? row[size_t(cy)] - first[size_t(cy)] : 0.0));   // height (mm)
            }
            ++n;
        }
    }
    m.samples = n;
    m.net = Mlp(nIn, nOut, opt.hidden, opt.seed);
    Mlp::Fit fit;
    fit.epochs = std::max(1, opt.epochs);
    fit.lr = opt.lr;
    fit.seed = opt.seed;
    if (progress) fit.progress = [&](int epoch, double loss) { return progress(epoch + 1, fit.epochs, loss); };
    if (!m.net.fit(X, Y, n, fit)) return fail("stopped");
    *out = std::move(m);
    return true;
}

std::vector<double> AnimationModel::weights(const QString& clip, const QString& other, double blend) const {
    const int a = clip.isEmpty() ? 0 : int(clips.indexOf(clip));
    const int b = other.isEmpty() ? -1 : int(clips.indexOf(other));
    if (a < 0 || a >= clips.size() || (!other.isEmpty() && b < 0)) return {};
    std::vector<double> w(size_t(clips.size()), 0.0);
    blend = b < 0 ? 0.0 : std::clamp(blend, 0.0, 1.0);
    w[size_t(a)] += 1 - blend;
    if (b >= 0) w[size_t(b)] += blend;
    return w;
}

double AnimationModel::durationOf(const std::vector<double>& w) const {
    double d = 0, total = 0;
    for (size_t i = 0; i < w.size() && i < durations.size(); ++i) { d += w[i] * durations[i]; total += w[i]; }
    return total > 0 ? d / total : 0.0;
}

take::Take AnimationModel::generate(const std::vector<double>& w, double duration, double rate,
                                    double x0, double y0, double z0, double heading0) const
{
    take::Take t;
    t.robot = robot;
    t.rate = rate > 0 ? rate : 50;
    if (w.size() != size_t(clips.size()) || net.empty()) return t;
    if (!(duration > 0)) duration = durationOf(w);
    duration = std::max(duration, 1e-3);
    const int n = std::max(2, int(std::lround(duration * t.rate)) + 1);
    const int J = int(joints.size());
    t.columns << "t";
    if (place) t.columns << "x" << "y" << "z" << "heading";
    t.columns << joints;
    std::vector<float> x(static_cast<size_t>(net.nIn())), y(static_cast<size_t>(net.nOut()));
    const double h0 = qDegreesToRadians(heading0), c0 = std::cos(h0), s0 = std::sin(h0);
    for (int i = 0; i < n; ++i) {
        const double tau = double(i) / (n - 1);
        inputs(w, tau, x.data());
        net.predict(x.data(), y.data());
        std::vector<double> row;
        row.reserve(size_t(t.columns.size()));
        row.push_back(tau * duration);
        if (place) {
            const double fwd = y[size_t(J)], left = y[size_t(J + 1)];
            row.push_back(x0 + fwd * c0 - left * s0);
            row.push_back(y0 + y[size_t(J + 3)]);
            row.push_back(z0 - fwd * s0 - left * c0);
            row.push_back(heading0 + y[size_t(J + 2)]);
        }
        for (int j = 0; j < J; ++j) row.push_back(y[size_t(j)]);
        t.rows.push_back(std::move(row));
    }
    return t;
}

QJsonObject AnimationModel::toJson() const {
    if (!m_packed.isEmpty()) return m_packed;
    QJsonArray d;
    for (double v : durations) d.append(v);
    m_packed = QJsonObject{{"kind", "animation"}, {"clips", QJsonArray::fromStringList(clips)}, {"durations", d},
                           {"joints", QJsonArray::fromStringList(joints)}, {"robot", robot}, {"place", place},
                           {"harmonics", harmonics}, {"samples", samples}, {"net", net.toJson()}};
    return m_packed;
}

bool AnimationModel::fromJson(const QJsonObject& o, AnimationModel* out) {
    AnimationModel m;
    for (const QJsonValue& v : o["clips"].toArray()) m.clips << v.toString();
    for (const QJsonValue& v : o["durations"].toArray()) m.durations.push_back(v.toDouble());
    for (const QJsonValue& v : o["joints"].toArray()) m.joints << v.toString();
    m.robot = o["robot"].toString();
    m.place = o["place"].toBool();
    m.harmonics = o["harmonics"].toInt(12);
    m.samples = o["samples"].toInt();
    if (m.clips.isEmpty() || m.durations.size() != size_t(m.clips.size()) || m.joints.isEmpty() || m.harmonics < 1)
        return false;
    if (!Mlp::fromJson(o["net"].toObject(), &m.net)) return false;
    if (m.net.nIn() != int(m.clips.size()) + 2 * m.harmonics + 1 ||
        m.net.nOut() != int(m.joints.size()) + (m.place ? 4 : 0))
        return false;
    m.m_packed = o;
    *out = std::move(m);
    return true;
}

} // namespace learn
