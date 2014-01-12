/*
 * Filename: Example1.cpp
 *   Author: Igor Makhtes
 *     Date: Jan 8, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>

#include <decision_making/FSM.h>
#include <decision_making/BT.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

const int TR_REPEAT_TASK = 10;

//BT_BGN(D11)
//{
//    BT_CALL_TASK(CreateLocalGoal);
//    BT_CALL_TASK(GoToLandmark);
//
//    BT_SEL_BGN(E1)
//    {
//        BT_SEQ_BGN(F1)
//        {
//            BT_CALL_TASK(IsGoal);
//            BT_CALL_TASK(ReachedGoal);
//        }
//        BT_SEQ_END(F1);
//
//        // Reference to D1
//
//        //BT_CALL_BT(D1);
//
//        BT_TASK_BGN(RETURN_REPEAT)
//        {
//            BT_TASK_RESULT(TaskResult::FAIL(TR_REPEAT_TASK, "REPEAT D1"));
//        }
//        BT_TASK_END(RETURN_REPEAT);
//    }
//    BT_SEL_END(E1);
//}
//BT_END(D11);
//
//
//BT_BGN(D1)
//{
//    BT_TASK_BGN(WHILE_RETURN_REPEAT)
//    {
//        TaskResult res;
//        do{
//            BT_CALL_BT(D11);
//            res = BT_LAST_NODE->run();
//        }while(res.error() == TR_REPEAT_TASK);
//    }
//    BT_TASK_END(WHILE_RETURN_REPEAT);
//}
//BT_END(D1);
//
//
//
//BT_BGN(C1)
//{
//    BT_CALL_TASK(MakeGlobalPath);
//    BT_CALL_BT(D1);
//}
//BT_END(C1);
//
//
//
//BT_BGN(SemanticNavigation)
//{
//    BT_SEQ_BGN(SemanticNavigationRoot)
//    {
//        BT_CALL_TASK(WaitingForMainGoal);
//        BT_SEL_BGN(A1)
//        {
//            BT_SEQ_BGN(B1)
//            {
//                BT_CALL_TASK(IsLocalized);
//                BT_CALL_BT(C1);
//            }
//            BT_SEQ_END(B1);
//
//            BT_SEQ_BGN(B2)
//            {
//                BT_CALL_TASK(LookForLandmark);
//
//                BT_CALL_BT(D1);
//            }
//            BT_SEQ_END(B2);
//
//            BT_CALL_BT(C1);
//        }
//        BT_SEL_END(A1);
//    }
//    BT_SEQ_END(SemanticNavigationRoot);
//}
//BT_END(SemanticNavigation);



BT_BGN(Decorators)
{
    BT_SEQ_BGN(RootElement)
    {
        BT_DEC_SUCCESS_BGN
        {
            BT_CALL_TASK(Task11);
            BT_CALL_TASK(Task12);
            BT_CALL_TASK(Task13);
//            BT_SET_TASK_RESULT(1);
        }
        BT_DEC_SUCCESS_END

        BT_DEC_NOT_BGN
        {
            BT_CALL_TASK(Task21);
            BT_CALL_TASK(Task22);
            BT_CALL_TASK(Task23);
//            BT_SET_TASK_RESULT_AFTER(TaskResult::SUCCESS, 1);
        }
        BT_DEC_NOT_END

        BT_DEC_FAIL_BGN(100)
        {
            BT_CALL_TASK(Task31);
            BT_CALL_TASK(Task32);
            BT_CALL_TASK(Task33);
//            BT_SET_TASK_RESULT(TaskResult::SUCCESS);
        }
        BT_DEC_FAIL_END
    }
    BT_SEQ_END(RootElement);
}
BT_END(Decorators);





//
//BT_BGN(SemanticNavigation)
//{
//    BT_SEQ_BGN(SemanticNavigationRoot)
//    {
//        BT_CALL_TASK(WaitingForMainGoal);
//        BT_SEL_BGN(A1)
//        {
//            BT_SEQ_BGN(B1)
//            {
//                BT_CALL_TASK(IsLocalized);
//                BT_SEQ_BGN(C1)
//                {
//                    BT_CALL_TASK(MakeGlobalPath);
//                    BT_SEQ_BGN(D1)
//                    {
//                        BT_CALL_TASK(CreateLocalGoal);
//                        BT_CALL_TASK(GoToLandmark);
//
//                        BT_SEL_BGN(E1)
//                        {
//                            BT_SEQ_BGN(F1)
//                            {
//                                BT_CALL_TASK(IsGoal);
//                                BT_CALL_TASK(ReachedGoal);
//                            }
//                            BT_SEQ_END(F1);
//
//                            // Reference to D1
//                        }
//                        BT_SEL_END(E1);
//                    }
//                    BT_SEQ_END(D1);
//                }
//                BT_SEQ_END(C1);
//            }
//            BT_SEQ_END(B1);
//
//            BT_SEQ_BGN(B2)
//            {
//                BT_CALL_TASK(LookForLandmark);
//
//                // Reference to D1
//            }
//            BT_SEQ_END(B2);
//
//            // Reference to C1
//        }
//        BT_SEL_END(A1);
//    }
//    BT_SEQ_END(SemanticNavigationRoot);
//}
//BT_END(SemanticNavigation);










FSM(SemanticNavigationFSM)
{
    FSM_STATES
    {
        WaitingForMainGoal,
        IsLocalized,
        MakeGlobalPath,
        CreateLocalGoal,
        GoToLandmark,
        IsGoal,
        LookForLandmark,
        ReachedGoal
    }
    FSM_START(WaitingForMainGoal)
    FSM_BGN
    {
/// WaitingForMainGoal
        FSM_STATE(WaitingForMainGoal)
        {
            FSM_CALL_TASK(WaitingForMainGoal)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/RECEIVED_GOAL, FSM_NEXT(IsLocalized))
            }
        }
/// IsLocalized
        FSM_STATE(IsLocalized)
        {
            FSM_CALL_TASK(IsLocalized)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/KNOWN_LOC, FSM_NEXT(MakeGlobalPath))
        FSM_ON_EVENT(/NOT_KNOWN_LOC, FSM_NEXT(LookForLandmark))
            }
        }
/// MakeGlobalPath
        FSM_STATE(MakeGlobalPath)
        {
            FSM_CALL_TASK(MakeGlobalPath)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/PATH_IS_RDY, FSM_NEXT(CreateLocalGoal))
            }
        }
/// CreateLocalGoal
        FSM_STATE(CreateLocalGoal)
        {
            FSM_CALL_TASK(CreateLocalGoal)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/LOCAL_GOAL_UPDATED, FSM_NEXT(GoToLandmark))
            }
        }
/// GoToLandmark
        FSM_STATE(GoToLandmark)
        {
            FSM_CALL_TASK(GoToLandmark)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/REACHED_LANDMARK, FSM_NEXT(IsGoal))
        FSM_ON_EVENT(/LOST, FSM_NEXT(LookForLandmark))
            }
        }
/// IsGoal
        FSM_STATE(IsGoal)
        {
            FSM_CALL_TASK(IsGoal)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/IS_GOAL, FSM_NEXT(ReachedGoal))
        FSM_ON_EVENT(/NOT_GOAL, FSM_NEXT(CreateLocalGoal))
            }
        }
/// LookForLandmark
        FSM_STATE(LookForLandmark)
        {
            FSM_CALL_TASK(LookForLandmark)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/FOUND_UNEXPECTED_LANDMARK, FSM_NEXT(MakeGlobalPath))
        FSM_ON_EVENT(/FOUND_EXPECTED_LANDMARK, FSM_NEXT(CreateLocalGoal))
            }
        }
/// ReachedGoal
        FSM_STATE(ReachedGoal)
        {
            FSM_CALL_TASK(ReachedGoal)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/FINISHED, FSM_NEXT(WaitingForMainGoal))
            }
        }
    }
    FSM_END
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "bt_example1");
    ros_decision_making_init(argc, argv);

    ros::AsyncSpinner spinner(2);
    spinner.start();

	return 0;
}
