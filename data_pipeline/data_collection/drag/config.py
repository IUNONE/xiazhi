"""RealMan drag mode identifiers shared by configuration and ROS control."""

FORCE_DRAG_MODE_VALUES = {"fast": 0, "precise": 1}
FORCE_DRAG_MODE_NAMES = {0: "fast", 1: "precise"}
DRAG_MODE_NAMES = {
    0: "电流环",
    1: "末端六维力-只动位置",
    2: "末端六维力-只动姿态",
    3: "末端六维力-位置和姿态",
}
