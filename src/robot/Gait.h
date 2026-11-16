#pragma once
#include <QMatrix4x4>
#include <QPointF>
#include <QString>
#include <QVector3D>
#include <map>
#include <optional>
#include <vector>

class SceneGraph;

// Walking for legged robots (the humanoids the Humanoid Builder makes, or any
// model with a left and a right foot below one root object). Kinematic: the
// robot's root object is carried along the way, each foot is placed on the
// floor where the plan puts it and inverse kinematics bends the legs to reach
// it. A foot on the floor stays exactly where it was set down. Nothing
// balances: the root is moved, so the robot cannot fall.
//
// Metres and radians here (the API converts). The floor is the root's height
// when the walk starts; a heading turns about +Y, 0 faces +X, and positive is
// to the robot's left (its −Z side), like an object's Y rotation.
namespace gait {

// Where a robot stands on the floor
struct Place {
    double x = 0, z = 0;    // m
    double heading = 0;     // rad about +Y
};
Place      placeOf(const QMatrix4x4& frame);            // an upright frame's spot and heading
QMatrix4x4 frameOf(const Place& p, double y = 0);       // translate (x, y, z) · rotate heading about Y
// `p` moved by `forward` and `left` (m) in its own frame while turning by
// `turn` (rad); the move follows the mean heading, so a turn makes an arc
Place      advance(const Place& p, double forward, double left, double turn);
double     angleDiff(double a, double b);               // a − b wrapped into (−π, π]

// The legs of a walking robot. Side 0 is the left, 1 the right.
struct Legs {
    int root = -1;                  // the object carried along (the robot's root)
    int foot[2] = {-1, -1};         // foot frames (at the ankle)
    std::vector<int> chain[2];      // the actuated joints that move each foot alone
    int arm[2] = {-1, -1};          // shoulder pitch joints for the arm swing, -1 = none
    QMatrix4x4 footHome[2];         // each foot frame relative to the robot's floor frame, every leg joint at 0
    QVector3D  hip[2];              // each leg's top joint pivot in the floor frame
    double legLength   = 0;         // hip to ankle, m
    double hipWidth    = 0;         // between the feet at home, m
    double ankleHeight = 0;         // floor to the ankle at home, m
    double footLength  = 0;         // the sole's size, m (drawing footprints)
    double footWidth   = 0;
    QVector3D soleCentre;           // the sole's centre in the foot frame (on the floor)
};

// Finds the feet below `root`: objects called "Left Foot" and "Right Foot"
// (any case, or any names holding left/right and foot), or the ones given.
// Each foot needs at least three actuated joints of its own (hip, knee,
// ankle). Returns false and says why when the robot can't walk.
bool findLegs(SceneGraph& scene, int root, Legs* legs, QString* error, int leftFoot = -1, int rightFoot = -1);
// Whether `root` has legs to walk on (cheap enough for listings)
bool canWalk(SceneGraph& scene, int root);

// How to walk. A value ≤ 0 (or < 0 where 0 is meaningful) means "from the legs".
struct Params {
    double stepLength  = 0;    // m between successive footprints along the way
    double sideStep    = 0;    // m at most sideways per step
    double turnPerStep = 0;    // rad at most per step
    double stepHeight  = 0;    // m the swinging foot lifts
    double stepTime    = 0;    // s per step
    double speed       = 0;    // m/s; when given, sets stepTime from stepLength
    double crouch      = -1;   // m the hips sink while walking (they must, to step)
    double sway        = -1;   // m the hips shift over the standing foot
    double armSwing    = -1;   // rad each arm swings (0 = arms keep still)
    double doubleSupport = 0.2;// share of each step with both feet down
};
// Fills the unset values from the size of the legs and checks the rest;
// returns what is wrong (empty = fine), and clamps
QString resolve(Params& p, const Legs& legs);

// A footprint: the foot `side` is set down where its home spot is when the
// robot stands at `at`
struct Footstep {
    int   side = 0;
    Place at;
};

// The body places (each at most a step from the one before) that walk
// `forward`, `left` and `turn` from `start`. The turn is spread over the
// walk, so walking with a turn follows an arc.
std::vector<Place> relativePlaces(const Place& start, double forward, double left, double turn, const Params& p);
// The body places along a path through `points` (world x, z), facing the
// way it goes and turning gradually at corners, then turning on the spot to
// `heading` when given
std::vector<Place> pathPlaces(const Place& start, const std::vector<QPointF>& points,
                              std::optional<double> heading, const Params& p);

struct Plan {
    Place  start;
    std::vector<Footstep> steps;   // the last one brings the feet together
    std::vector<Place>    body;    // where the body is after each step (body[0] = start)
    Params params;
    double prep = 0;               // s to crouch before the first step, and to stand after the last
    bool   crouchOnly = false;     // no steps, but crouch and stand (a walk stopped before its first step)
    double duration() const;
    Place  end() const { return body.empty() ? start : body.back(); }
};
// Footsteps through `places`: alternate feet, the first one towards the
// side the walk turns or goes (`firstSide` -1 = choose), then a step that
// brings the other foot alongside
Plan makePlan(const Place& start, const std::vector<Place>& places, const Params& p, int firstSide = -1);

// Runs a plan on a scene: puts the root and the legs where the plan has them
// at a time
class Walker {
public:
    Walker(Legs legs, Plan plan) : m_legs(std::move(legs)), m_plan(std::move(plan)) {}

    // Records the start: where the feet and the root are, the arms' values
    void begin(SceneGraph& scene);
    // Moves the root and solves both legs for `t` seconds after the start.
    // Fills the leg (and arm) joints' values. False when a foot can't reach
    // its spot (`error` = how far off it stays, m).
    bool apply(SceneGraph& scene, double t, std::map<int, double>* joints, double* error);
    // Ends the walk after the step under way at `t`, feet together
    void stopAfter(double t);

    double duration() const { return m_plan.duration(); }
    int    stepsDone(double t) const;
    int    stepCount() const { return int(m_plan.steps.size()); }
    const Plan& plan() const { return m_plan; }
    const Legs& legs() const { return m_legs; }
    std::vector<int> joints() const;   // every joint it moves

    // Drawing: each footprint's sole as a world frame (x along the foot,
    // on the floor), its side, and whether it is already set down at `t`
    struct Print { QMatrix4x4 frame; int side = 0; bool done = false; };
    std::vector<Print> prints(double t) const;

private:
    QMatrix4x4 target(int side, const Place& at) const;
    // How far a foot on the floor rolls (rad; > 0 onto its toes, < 0 onto its
    // heel) with the body `ahead` m in front of it, its next step `travel` m
    // along it and its hip `reach` (× the leg's length) from the flat foot's ankle
    double     toeOff(double ahead, double travel, double reach) const;
    QMatrix4x4 rolled(const QMatrix4x4& foot, double angle) const; // the foot rolled onto its toes (heel if < 0)

    Legs   m_legs;
    Plan   m_plan;
    double m_floorY = 0;
    QMatrix4x4 m_rootOffset;        // the root's frame relative to its floor frame at the start
    QMatrix4x4 m_startFoot[2];      // where the feet were
    double m_startArm[2] = {0, 0};
    double m_swingPitch = 0;        // rad the swinging foot's toes point down at the top of a step
    double m_pitchBias[2] = {0, 0}; // rad more the swinging foot pitches down, where its leg needs it
    double m_lastT = -1;
    double m_roll[2] = {0, 0};      // the roll each foot on the floor had last
    int    m_rollStep[2] = {-1, -1};// the step it was about to lift for (its swing starts from that roll)
};

} // namespace gait
