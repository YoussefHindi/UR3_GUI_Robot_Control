import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: qsTr("Robot Controller")

    Column {
        anchors.centerIn: parent

        Button {
            text: "Make Rectangle"
            onClicked: {
                myNode.startRosNode()
            }
        }

        Button {
            text: "Circle"
            onClicked: {
                myNode.makeCircle()
            }
        }
    }
}

