#pragma once
#include <QString>
#include <QStringList>
#include <functional>
#include <memory>
#include <vector>

// Small, safe formula language used throughout the automation system:
//
//   literals     1.5  true  false
//   symbols      Start   J1.pos   PickAndPlace.cycles   (resolved to slots)
//   operators    + - * / %   == != < <= > >=   && || !   ?:   ( )
//   functions    abs min max clamp sin cos tan sqrt floor ceil round sign
//                rising(x) falling(x) changed(x) time()
//
// Everything evaluates to a double; booleans are 0/1 and any non-zero value
// is truthy. Expressions are compiled once against a symbol resolver (name →
// slot index), so unknown names are reported at edit time rather than run time.
//
// Edge functions compare the argument evaluated against the current scan's
// values with the same argument evaluated against the previous scan's values,
// so they stay correct no matter how rarely the enclosing expression runs.

namespace expr {

struct Node;

struct EvalContext {
    const double* cur  = nullptr;   // symbol values for this scan
    const double* prev = nullptr;   // symbol values at the previous scan
    double        time = 0.0;       // simulation time, seconds
    double        dt   = 0.0;       // scan period, seconds
};

using Resolver = std::function<int(const QString& name)>;   // -1 = unknown

class Expression {
public:
    Expression() = default;

    // Parses and resolves `source`. An empty/whitespace-only source compiles
    // to an expression that is "empty" (see isEmpty) and evaluates to 0.
    bool compile(const QString& source, const Resolver& resolve, QString* error = nullptr);

    bool   isEmpty() const { return !m_root; }
    double eval(const EvalContext& ctx) const;
    bool   test(const EvalContext& ctx) const { return eval(ctx) != 0.0; }

    // Slot indices referenced by this expression (for dependency ordering)
    const std::vector<int>& dependencies() const { return m_deps; }

private:
    std::shared_ptr<Node> m_root;
    std::vector<int>      m_deps;
};

// `name = expr` statements separated by ';' or newlines — used for operation
// on-start / on-finish actions.
struct Assignment {
    QString    target;
    int        slot = -1;
    Expression value;
};

bool compileAssignments(const QString& source, const Resolver& resolveRead,
                        const Resolver& resolveWrite, std::vector<Assignment>& out,
                        QString* error = nullptr);

// Identifiers (symbol references, not function names) appearing in `source`.
// Best-effort: returns what could be tokenized even if the source has errors.
QStringList identifiers(const QString& source);

// Rewrites every symbol reference equal to `oldName`, or prefixed by
// `oldName + "."` (e.g. J1.pos when renaming actuator J1), to use `newName`.
QString renameIdentifier(const QString& source, const QString& oldName, const QString& newName);

// True when `name` is usable as a signal/actuator/sequence name.
bool isValidName(const QString& name);

// Built-in function names (for completion lists)
QStringList functionNames();

} // namespace expr
