#pragma once
#include "Take.h"
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <cstdint>
#include <functional>
#include <vector>

// Learning motion from takes, in the app (the same models as the Python
// library's motionstudio.learn, without NumPy).
namespace learn {

// A small fully connected network: tanh hidden layers, a linear output,
// inputs and outputs standardized inside. Trained with Adam on the mean
// squared error, the learning rate easing down to a tenth (cosine).
class Mlp {
public:
    Mlp() = default;
    Mlp(int nIn, int nOut, const std::vector<int>& hidden, uint32_t seed = 0);

    int  nIn() const { return m_layers.empty() ? 0 : m_layers.front().in; }
    int  nOut() const { return m_layers.empty() ? 0 : m_layers.back().out; }
    bool empty() const { return m_layers.empty(); }

    // One row of outputs for one row of inputs
    void predict(const float* x, float* y) const;

    struct Fit {
        int      epochs = 400;
        int      batch  = 128;
        double   lr     = 2e-3;
        uint32_t seed   = 0;
        // Called after each epoch with its mean squared error (standardized);
        // return false to stop
        std::function<bool(int epoch, double loss)> progress;
    };
    // Rows of inputs X (n × nIn) and targets Y (n × nOut). Returns false
    // when stopped by `progress` (the network is then partly trained).
    bool fit(const std::vector<float>& X, const std::vector<float>& Y, int n, const Fit& options);
    double loss() const { return m_loss; }      // the last epoch's

    QJsonObject toJson() const;
    static bool fromJson(const QJsonObject& o, Mlp* out);

private:
    struct Layer {
        int in = 0, out = 0;
        std::vector<float> W, b;    // W: in × out, row-major
    };
    std::vector<Layer> m_layers;
    std::vector<float> m_xMean, m_xStd, m_yMean, m_yStd;
    double m_loss = 0;
};

// Learns motions as clips: for each clip, the joint values and how the robot
// moves over the floor (ahead, to its left, its turn and height, from where
// the clip starts), over the clip's time from 0 to 1. It then plays a clip at
// any length, or a blend of clips.
class AnimationModel {
public:
    QStringList clips;              // the motions it learned, by name
    std::vector<double> durations;  // their lengths (s)
    QStringList joints;             // actuator names
    QString robot;                  // what the takes were recorded on
    bool    place = false;          // it learned how the robot moves over the floor
    int     harmonics = 12;
    int     samples = 0;            // rows it learned from
    Mlp     net;

    struct Options {
        std::vector<int> hidden{128, 128};
        int      harmonics = 12;
        int      epochs = 400;
        double   lr = 2e-3;
        uint32_t seed = 0;
    };
    // Learns the named takes; they must share their joints (one robot).
    // `progress(epoch, epochs, loss)` returns false to stop (then false,
    // with error "stopped").
    static bool train(const std::vector<std::pair<QString, take::Take>>& takes, const Options& options,
                      AnimationModel* out, QString* error = nullptr,
                      const std::function<bool(int, int, double)>& progress = {});

    // Weights per clip from a clip and, optionally, a second one blended in
    // by `blend` (0..1); empty when a name isn't a clip of this model
    std::vector<double> weights(const QString& clip, const QString& other = {}, double blend = 0) const;
    double durationOf(const std::vector<double>& weights) const;   // the blend of the clips' lengths
    // The clip (weights) as a take `duration` s long (≤ 0: its own) at `rate`
    // per second: t, with `place` x y z heading from `start` (mm, mm, mm,
    // degrees), then the joints
    take::Take generate(const std::vector<double>& weights, double duration = 0, double rate = 50,
                        double x0 = 0, double y0 = 0, double z0 = 0, double heading0 = 0) const;

    QJsonObject toJson() const;
    static bool fromJson(const QJsonObject& o, AnimationModel* out);

private:
    void inputs(const std::vector<double>& weights, double tau, float* x) const;
    mutable QJsonObject m_packed;   // toJson's cache: the model never changes once trained
};

} // namespace learn
