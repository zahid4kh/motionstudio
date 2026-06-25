#pragma once
#include "SceneObject.h"
#include "Joint.h"
#include "Motor.h"
#include <QObject>
#include <QMatrix4x4>
#include <unordered_map>
#include <vector>

class SceneGraph : public QObject {
    Q_OBJECT
public:
    explicit SceneGraph(QObject* parent = nullptr);

    // Objects
    int              addObject(const QString& name, PrimitiveType type, int parentId = -1);
    void             removeObject(int id);
    SceneObject*     getObject(int id);
    const SceneObject* getObject(int id) const;
    const std::vector<int>&                       rootObjects() const { return m_roots; }
    const std::unordered_map<int, SceneObject>&   allObjects()  const { return m_objects; }

    // Joints
    int     addJoint(Joint joint);
    void    removeJoint(int id);
    Joint*  getJoint(int id);
    const Joint* getJoint(int id) const;
    const std::unordered_map<int, Joint>& allJoints() const { return m_joints; }
    Joint*  jointForChild(int childBodyId);

    // Motors
    int     addMotor(Motor motor);
    void    removeMotor(int id);
    Motor*  getMotor(int id);
    const std::unordered_map<int, Motor>& allMotors() const { return m_motors; }

    // Selection
    void selectObject(int id);
    void deselectAll();
    int  selectedObjectId() const { return m_selectedId; }

    // Transform helpers
    QMatrix4x4 worldTransform(int id) const;
    QMatrix4x4 localTransform(int id) const;

    void clear();

signals:
    void objectAdded(int id);
    void objectRemoved(int id);
    void objectChanged(int id);
    void selectionChanged(int id);
    void jointAdded(int id);
    void motorAdded(int id);
    void sceneCleared();

private:
    std::unordered_map<int, SceneObject> m_objects;
    std::unordered_map<int, Joint>       m_joints;
    std::unordered_map<int, Motor>       m_motors;
    std::vector<int>                     m_roots;
    int  m_nextId    = 1;
    int  m_selectedId = -1;
};
