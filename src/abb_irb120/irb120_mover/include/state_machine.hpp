#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

namespace IRBStateMachine
{
typedef enum
{
    Initialization = 0,
    DetectPCB = 1,
    Move2DetectSOIC = 2,
    DetectSOIC = 3,
    Move2PickSyringe = 4,
    PickSyringe = 5,
    Move2ReleaseSolderPaste = 6,
    ApplySolderPaste = 7,
    Move2DropSyringe = 8,
    DropSyringe = 9,
    Move2PickSuction = 10,
    PickSuction = 11,
    Move2PickSOIC = 12,
    PickSOIC = 13,
    Move2PlaceSOIC = 14,
    PlaceSOIC = 15,
    Move2DropSuction = 16,
    DropSuction = 17,
    Move2PickHotAirPencil = 18,
    PickHotAirPencil = 19,
    Move2SolderPCB = 20,
    ApplyHotAir = 21,
    Move2DropHotAirPencil = 22,
    DropHotAirPencil = 23,
    ReturnHome = 24
} RobotState;
}
#endif
