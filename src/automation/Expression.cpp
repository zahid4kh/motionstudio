#include "Expression.h"
#include <QRegularExpression>
#include <algorithm>
#include <cmath>

namespace expr {

// ── Lexer ────────────────────────────────────────────────────────────────────

namespace {

enum class Tok { End, Number, Ident, Op, LParen, RParen, Comma, Question, Colon, Assign, Semi, Error };

struct Token {
    Tok     type = Tok::End;
    QString text;
    double  number = 0.0;
    int     pos = 0;
};

bool isIdentStart(QChar c) { return c.isLetter() || c == '_'; }
bool isIdentChar (QChar c) { return c.isLetterOrNumber() || c == '_'; }

std::vector<Token> tokenize(const QString& s, QString* error) {
    std::vector<Token> out;
    int i = 0;
    const int n = s.size();
    auto fail = [&](const QString& msg, int pos) {
        if (error) *error = QString("%1 (at column %2)").arg(msg).arg(pos + 1);
        Token t; t.type = Tok::Error; t.pos = pos;
        out.push_back(t);
    };
    while (i < n) {
        QChar c = s[i];
        if (c.isSpace()) { ++i; continue; }
        // '//' line comments
        if (c == '/' && i + 1 < n && s[i + 1] == '/') {
            while (i < n && s[i] != '\n') ++i;
            continue;
        }
        Token t; t.pos = i;
        if (c.isDigit() || (c == '.' && i + 1 < n && s[i + 1].isDigit())) {
            int st = i;
            while (i < n && (s[i].isDigit() || s[i] == '.')) ++i;
            if (i < n && (s[i] == 'e' || s[i] == 'E')) {
                int save = i;
                ++i;
                if (i < n && (s[i] == '+' || s[i] == '-')) ++i;
                if (i < n && s[i].isDigit()) { while (i < n && s[i].isDigit()) ++i; }
                else i = save;
            }
            bool ok = false;
            t.type = Tok::Number;
            t.text = s.mid(st, i - st);
            t.number = t.text.toDouble(&ok);
            if (!ok) { fail("Bad number '" + t.text + "'", st); return out; }
            out.push_back(t);
            continue;
        }
        if (isIdentStart(c)) {
            int st = i;
            // Dotted names: J1.pos, Seq.cycles
            while (i < n && (isIdentChar(s[i]) || (s[i] == '.' && i + 1 < n && isIdentStart(s[i + 1])))) ++i;
            t.type = Tok::Ident;
            t.text = s.mid(st, i - st);
            out.push_back(t);
            continue;
        }
        auto two = s.mid(i, 2);
        if (two == "&&" || two == "||" || two == "==" || two == "!=" || two == "<=" || two == ">=") {
            t.type = Tok::Op; t.text = two; i += 2; out.push_back(t); continue;
        }
        switch (c.unicode()) {
            case '+': case '-': case '*': case '/': case '%': case '<': case '>': case '!':
                t.type = Tok::Op; t.text = QString(c); break;
            case '(': t.type = Tok::LParen; break;
            case ')': t.type = Tok::RParen; break;
            case ',': t.type = Tok::Comma; break;
            case '?': t.type = Tok::Question; break;
            case ':': t.type = Tok::Colon; break;
            case '=': t.type = Tok::Assign; break;
            case ';': t.type = Tok::Semi; break;
            default:
                fail(QString("Unexpected character '%1'").arg(c), i);
                return out;
        }
        if (t.text.isEmpty()) t.text = QString(c);
        ++i;
        out.push_back(t);
    }
    Token e; e.type = Tok::End; e.pos = n;
    out.push_back(e);
    return out;
}

} // namespace

// ── AST ──────────────────────────────────────────────────────────────────────

enum class NodeKind { Const, Symbol, Unary, Binary, Ternary, Call };

struct Node {
    NodeKind kind = NodeKind::Const;
    double   value = 0.0;      // Const
    int      slot  = -1;       // Symbol
    QString  op;               // Unary/Binary operator or Call function name
    std::vector<std::shared_ptr<Node>> args;
};

namespace {

struct FuncInfo { const char* name; int minArgs; int maxArgs; };
const FuncInfo kFuncs[] = {
    {"abs", 1, 1}, {"min", 2, 16}, {"max", 2, 16}, {"clamp", 3, 3},
    {"sin", 1, 1}, {"cos", 1, 1}, {"tan", 1, 1}, {"sqrt", 1, 1},
    {"floor", 1, 1}, {"ceil", 1, 1}, {"round", 1, 1}, {"sign", 1, 1},
    {"rising", 1, 1}, {"falling", 1, 1}, {"changed", 1, 1}, {"time", 0, 0},
};

const FuncInfo* findFunc(const QString& name) {
    for (const auto& f : kFuncs)
        if (name == QLatin1String(f.name)) return &f;
    return nullptr;
}

class Parser {
public:
    Parser(const std::vector<Token>& toks, const Resolver& res, std::vector<int>& deps)
        : m_t(toks), m_res(res), m_deps(deps) {}

    QString error;

    const Token& peek() const { return m_t[size_t(m_i)]; }
    const Token& next() { return m_t[size_t(m_i++)]; }
    bool atEnd() const { return peek().type == Tok::End; }
    int  index() const { return m_i; }

    bool isOp(const char* op) const { return peek().type == Tok::Op && peek().text == QLatin1String(op); }

    void fail(const QString& msg, int pos) {
        if (error.isEmpty()) error = QString("%1 (at column %2)").arg(msg).arg(pos + 1);
    }

    std::shared_ptr<Node> parseExpr() { return parseTernary(); }

private:
    std::shared_ptr<Node> parseTernary() {
        auto cond = parseBinary(0);
        if (!cond) return nullptr;
        if (peek().type == Tok::Question) {
            next();
            auto a = parseTernary();
            if (!a) return nullptr;
            if (peek().type != Tok::Colon) { fail("Expected ':' in conditional", peek().pos); return nullptr; }
            next();
            auto b = parseTernary();
            if (!b) return nullptr;
            auto n = std::make_shared<Node>();
            n->kind = NodeKind::Ternary;
            n->args = {cond, a, b};
            return n;
        }
        return cond;
    }

    static int precedence(const QString& op) {
        if (op == "||") return 1;
        if (op == "&&") return 2;
        if (op == "==" || op == "!=") return 3;
        if (op == "<" || op == "<=" || op == ">" || op == ">=") return 4;
        if (op == "+" || op == "-") return 5;
        if (op == "*" || op == "/" || op == "%") return 6;
        return -1;
    }

    std::shared_ptr<Node> parseBinary(int minPrec) {
        auto lhs = parseUnary();
        if (!lhs) return nullptr;
        while (peek().type == Tok::Op) {
            QString op = peek().text;
            int p = precedence(op);
            if (p < 0 || p < minPrec) break;
            next();
            auto rhs = parseBinary(p + 1);
            if (!rhs) return nullptr;
            auto n = std::make_shared<Node>();
            n->kind = NodeKind::Binary;
            n->op   = op;
            n->args = {lhs, rhs};
            lhs = n;
        }
        return lhs;
    }

    std::shared_ptr<Node> parseUnary() {
        if (isOp("-") || isOp("!") || isOp("+")) {
            QString op = next().text;
            auto a = parseUnary();
            if (!a) return nullptr;
            if (op == "+") return a;
            auto n = std::make_shared<Node>();
            n->kind = NodeKind::Unary;
            n->op   = op;
            n->args = {a};
            return n;
        }
        return parsePrimary();
    }

    std::shared_ptr<Node> parsePrimary() {
        const Token& t = peek();
        switch (t.type) {
            case Tok::Number: {
                next();
                auto n = std::make_shared<Node>();
                n->kind = NodeKind::Const;
                n->value = t.number;
                return n;
            }
            case Tok::LParen: {
                next();
                auto e = parseExpr();
                if (!e) return nullptr;
                if (peek().type != Tok::RParen) { fail("Expected ')'", peek().pos); return nullptr; }
                next();
                return e;
            }
            case Tok::Ident: {
                Token id = next();
                if (peek().type == Tok::LParen) return parseCall(id);
                if (id.text == "true" || id.text == "false") {
                    auto n = std::make_shared<Node>();
                    n->kind = NodeKind::Const;
                    n->value = id.text == "true" ? 1.0 : 0.0;
                    return n;
                }
                int slot = m_res ? m_res(id.text) : -1;
                if (slot < 0) { fail("Unknown name '" + id.text + "'", id.pos); return nullptr; }
                if (std::find(m_deps.begin(), m_deps.end(), slot) == m_deps.end())
                    m_deps.push_back(slot);
                auto n = std::make_shared<Node>();
                n->kind = NodeKind::Symbol;
                n->slot = slot;
                return n;
            }
            case Tok::End:
                fail("Unexpected end of expression", t.pos);
                return nullptr;
            default:
                fail("Unexpected '" + t.text + "'", t.pos);
                return nullptr;
        }
    }

    std::shared_ptr<Node> parseCall(const Token& id) {
        const FuncInfo* f = findFunc(id.text);
        if (!f) { fail("Unknown function '" + id.text + "'", id.pos); return nullptr; }
        next(); // '('
        auto n = std::make_shared<Node>();
        n->kind = NodeKind::Call;
        n->op   = id.text;
        if (peek().type != Tok::RParen) {
            while (true) {
                auto a = parseExpr();
                if (!a) return nullptr;
                n->args.push_back(a);
                if (peek().type == Tok::Comma) { next(); continue; }
                break;
            }
        }
        if (peek().type != Tok::RParen) { fail("Expected ')' after arguments", peek().pos); return nullptr; }
        next();
        int na = int(n->args.size());
        if (na < f->minArgs || na > f->maxArgs) {
            fail(QString("%1() takes %2 argument%3").arg(id.text)
                     .arg(f->minArgs == f->maxArgs ? QString::number(f->minArgs)
                                                   : QString("%1–%2").arg(f->minArgs).arg(f->maxArgs))
                     .arg(f->maxArgs == 1 ? "" : "s"), id.pos);
            return nullptr;
        }
        return n;
    }

    const std::vector<Token>& m_t;
    const Resolver&           m_res;
    std::vector<int>&         m_deps;
    int                       m_i = 0;
};

double evalNode(const Node* n, const EvalContext& c, bool usePrev);

double callFunc(const Node* n, const EvalContext& c, bool usePrev) {
    const QString& f = n->op;
    auto a = [&](int i) { return evalNode(n->args[size_t(i)].get(), c, usePrev); };
    if (f == "abs")   return std::fabs(a(0));
    if (f == "sqrt")  return std::sqrt(std::max(0.0, a(0)));
    if (f == "sin")   return std::sin(a(0) * M_PI / 180.0);   // degrees, like the rest of the app
    if (f == "cos")   return std::cos(a(0) * M_PI / 180.0);
    if (f == "tan")   return std::tan(a(0) * M_PI / 180.0);
    if (f == "floor") return std::floor(a(0));
    if (f == "ceil")  return std::ceil(a(0));
    if (f == "round") return std::round(a(0));
    if (f == "sign")  { double v = a(0); return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0); }
    if (f == "min" || f == "max") {
        double r = a(0);
        for (size_t i = 1; i < n->args.size(); ++i) {
            double v = a(int(i));
            r = (f == "min") ? std::min(r, v) : std::max(r, v);
        }
        return r;
    }
    if (f == "clamp") return std::clamp(a(0), std::min(a(1), a(2)), std::max(a(1), a(2)));
    if (f == "time")  return usePrev ? c.time - c.dt : c.time;
    if (f == "rising" || f == "falling" || f == "changed") {
        // Previous-scan evaluation of an edge function itself is meaningless; treat as false.
        if (usePrev || !c.prev) return 0.0;
        double now  = evalNode(n->args[0].get(), c, false);
        double then = evalNode(n->args[0].get(), c, true);
        if (f == "rising")  return (now != 0.0 && then == 0.0) ? 1.0 : 0.0;
        if (f == "falling") return (now == 0.0 && then != 0.0) ? 1.0 : 0.0;
        return now != then ? 1.0 : 0.0;
    }
    return 0.0;
}

double evalNode(const Node* n, const EvalContext& c, bool usePrev) {
    switch (n->kind) {
        case NodeKind::Const:  return n->value;
        case NodeKind::Symbol: {
            const double* vals = usePrev && c.prev ? c.prev : c.cur;
            return vals ? vals[n->slot] : 0.0;
        }
        case NodeKind::Unary: {
            double v = evalNode(n->args[0].get(), c, usePrev);
            return n->op == "-" ? -v : (v == 0.0 ? 1.0 : 0.0);
        }
        case NodeKind::Binary: {
            const QString& op = n->op;
            // Short-circuit logic
            if (op == "&&") return (evalNode(n->args[0].get(), c, usePrev) != 0.0 &&
                                    evalNode(n->args[1].get(), c, usePrev) != 0.0) ? 1.0 : 0.0;
            if (op == "||") return (evalNode(n->args[0].get(), c, usePrev) != 0.0 ||
                                    evalNode(n->args[1].get(), c, usePrev) != 0.0) ? 1.0 : 0.0;
            double l = evalNode(n->args[0].get(), c, usePrev);
            double r = evalNode(n->args[1].get(), c, usePrev);
            switch (op[0].unicode()) {
                case '+': return l + r;
                case '-': return l - r;
                case '*': return l * r;
                case '/': return r == 0.0 ? 0.0 : l / r;
                case '%': return r == 0.0 ? 0.0 : std::fmod(l, r);
                case '<': return (op.size() == 2 ? l <= r : l < r) ? 1.0 : 0.0;
                case '>': return (op.size() == 2 ? l >= r : l > r) ? 1.0 : 0.0;
                case '=': return std::fabs(l - r) < 1e-9 ? 1.0 : 0.0;
                case '!': return std::fabs(l - r) < 1e-9 ? 0.0 : 1.0;
            }
            return 0.0;
        }
        case NodeKind::Ternary:
            return evalNode(n->args[0].get(), c, usePrev) != 0.0
                ? evalNode(n->args[1].get(), c, usePrev)
                : evalNode(n->args[2].get(), c, usePrev);
        case NodeKind::Call:
            return callFunc(n, c, usePrev);
    }
    return 0.0;
}

} // namespace

// ── Expression ───────────────────────────────────────────────────────────────

bool Expression::compile(const QString& source, const Resolver& resolve, QString* error) {
    m_root.reset();
    m_deps.clear();
    if (source.trimmed().isEmpty()) return true;

    QString err;
    auto toks = tokenize(source, &err);
    if (!err.isEmpty()) { if (error) *error = err; return false; }

    Parser p(toks, resolve, m_deps);
    auto root = p.parseExpr();
    if (root && !p.atEnd()) p.fail("Unexpected '" + p.peek().text + "'", p.peek().pos);
    if (!p.error.isEmpty() || !root) {
        if (error) *error = p.error.isEmpty() ? QString("Invalid expression") : p.error;
        m_deps.clear();
        return false;
    }
    m_root = root;
    return true;
}

double Expression::eval(const EvalContext& ctx) const {
    if (!m_root) return 0.0;
    double v = evalNode(m_root.get(), ctx, false);
    return std::isfinite(v) ? v : 0.0;
}

// ── Assignments ──────────────────────────────────────────────────────────────

bool compileAssignments(const QString& source, const Resolver& resolveRead,
                        const Resolver& resolveWrite, std::vector<Assignment>& out,
                        QString* error) {
    out.clear();
    // Split on ';' and newlines, keeping column offsets for error messages.
    int start = 0;
    const int n = source.size();
    for (int i = 0; i <= n; ++i) {
        if (i < n && source[i] != ';' && source[i] != '\n') continue;
        QString stmt = source.mid(start, i - start);
        int offset = start;
        start = i + 1;
        if (stmt.trimmed().isEmpty()) continue;

        int eq = -1;
        for (int k = 0; k < stmt.size(); ++k) {
            if (stmt[k] != '=') continue;
            QChar before = k > 0 ? stmt[k - 1] : QChar();
            QChar after  = k + 1 < stmt.size() ? stmt[k + 1] : QChar();
            if (after == '=' || before == '=' || before == '!' || before == '<' || before == '>') continue;
            eq = k;
            break;
        }
        if (eq < 0) {
            if (error) *error = QString("Expected 'name = value' (at column %1)").arg(offset + 1);
            return false;
        }
        Assignment a;
        a.target = stmt.left(eq).trimmed();
        if (!isValidName(a.target)) {
            if (error) *error = QString("Invalid assignment target '%1'").arg(a.target);
            return false;
        }
        a.slot = resolveWrite ? resolveWrite(a.target) : -1;
        if (a.slot < 0) {
            if (error) *error = QString("'%1' is not a writable signal (Input or Internal)").arg(a.target);
            return false;
        }
        QString rhs = stmt.mid(eq + 1);
        if (rhs.trimmed().isEmpty()) {
            if (error) *error = QString("Missing value for '%1'").arg(a.target);
            return false;
        }
        QString err;
        if (!a.value.compile(rhs, resolveRead, &err)) {
            if (error) *error = a.target + " = …: " + err;
            return false;
        }
        out.push_back(std::move(a));
    }
    return true;
}

// ── Identifier utilities ─────────────────────────────────────────────────────

QStringList identifiers(const QString& source) {
    QStringList out;
    auto toks = tokenize(source, nullptr);
    for (size_t i = 0; i < toks.size(); ++i) {
        const Token& t = toks[i];
        if (t.type != Tok::Ident) continue;
        if (t.text == "true" || t.text == "false") continue;
        bool isCall = i + 1 < toks.size() && toks[i + 1].type == Tok::LParen;
        if (isCall) continue;
        if (!out.contains(t.text)) out << t.text;
    }
    return out;
}

QString renameIdentifier(const QString& source, const QString& oldName, const QString& newName) {
    if (oldName.isEmpty() || oldName == newName) return source;
    auto toks = tokenize(source, nullptr);
    QString out = source;
    // Replace back-to-front so earlier positions stay valid
    for (auto it = toks.rbegin(); it != toks.rend(); ++it) {
        if (it->type != Tok::Ident) continue;
        const QString& t = it->text;
        if (t == oldName)
            out.replace(it->pos, t.size(), newName);
        else if (t.startsWith(oldName + "."))
            out.replace(it->pos, oldName.size(), newName);
    }
    return out;
}

bool isValidName(const QString& name) {
    static const QRegularExpression re("^[A-Za-z_][A-Za-z0-9_]*$");
    if (!re.match(name).hasMatch()) return false;
    if (name == "true" || name == "false") return false;
    return findFunc(name) == nullptr;
}

QStringList functionNames() {
    QStringList out;
    for (const auto& f : kFuncs) out << QString::fromLatin1(f.name);
    return out;
}

} // namespace expr
