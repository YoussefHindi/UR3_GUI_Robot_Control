#ifndef MYNODE_H
#define MYNODE_H

// #include <rclcpp/rclcpp.hpp>
#include <QObject>

class MyNode : public QObject
{
    Q_OBJECT
public:
    explicit MyNode(QObject *parent = nullptr);

public slots:
    void startRosNode();
    void makeCircle();

private:
    // rclcpp::Node::SharedPtr node_;
};

#endif // MYNODE_H

