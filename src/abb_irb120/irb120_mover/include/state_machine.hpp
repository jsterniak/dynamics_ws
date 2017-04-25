#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

namespace IRBStateMachine
{
    typedef enum
    {
        Initialization                  = 0,
        DetectPCB                       = 1,
        Move2DetectSOIC                 = 2,
        DetectSOIC                      = 3,
        Move2PickSyringe                = 4,
        PickSyringe                     = 5,
        Move2ReleaseSolderPaste         = 6,
        ApplySolderPasteFirstSide       = 7,
	    ApplySolderPasteMoveSide        = 8,
	    ApplySolderPasteSecondSide      = 9,
        Move2DropSyringe                = 10,
        DropSyringe                     = 11,
        Move2PickSuction                = 12,
        PickSuction                     = 13,
        Move2PickSOIC                   = 14,
        PickSOIC                        = 15,
        Move2PlaceSOIC                  = 16,
        PlaceSOIC                       = 17,
        Move2DropSuction                = 18,
        DropSuction                     = 19,
        Move2PickHotAirPencil           = 20,
        PickHotAirPencil                = 21,
        Move2SolderPCB                  = 22,
        ApplyHotAir                     = 23,
        Move2DropHotAirPencil           = 24,
        DropHotAirPencil                = 25,
        ReturnHome                      = 26
    } RobotState;
}
#endif
