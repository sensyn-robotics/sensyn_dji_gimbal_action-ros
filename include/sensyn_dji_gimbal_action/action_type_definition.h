
// #ifndef ELAS_ACTION_TYPE_DIFINITION_H
// #define ELAS_ACTION_TYPE_DIFINITION_H
// with pose
enum class GimbalCameraActionType : int {
    GoToPoint, // position , time=0
    FollowPath, // path
    Stop, // time =x ,hovering
    Turn, // position,rotation,time=0
    Stay, // position,rotation,time=x
    Land,
    TakeOff,
    AimCamera,
    TargetCamera,
    TakePicture,
    RecordVideo,
    StartRecordVideo,
    StopRecordVideo
};

std::map<int, std::string> map_GimbalCameraAction = {
        {0, "GoToPoint"},
        {1, "FollowPath"},
        {2, "Stop"},
        {3, "Turn"},
        {4, "Stay"},
        {5, "Land"},
        {6, "TakeOff"},
        {7, "AimCamera"},
        {8, "TargetCamera"},
        {9, "TakePicture"},
        {10, "RecordVideo"},
        {11, "StartRecordVideo"},
        {12, "StopRecordVideo"},

};

// enum class SensorAction_Type {
//     None,
//     Camera,
//     LiDAR
// };

// #endif //ELAS_ACTION_TYPE_DIFINITION_H
