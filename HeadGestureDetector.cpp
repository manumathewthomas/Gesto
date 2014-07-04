#include "ni/XnOpenNI.h"
#include "ni/XnCodecIDs.h"
#include "ni/XnCppWrapper.h"
#include "floatfann.h"
#include "fann.h"
#define SAMPLE_XML_PATH "/root/Kinect/nite/Data/Sample-Tracking.xml"
xn::Context g_Context;
xn::ScriptNode g_ScriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Recorder* g_pRecorder;


XnUserID g_nPlayer = 0;
XnBool g_bCalibrated = FALSE;
XnBool AssignPlayer(XnUserID user)
{
	if (g_nPlayer != 0)
		return FALSE;

	XnPoint3D com;
	g_UserGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return FALSE;

	printf("Matching for existing calibration\n");
	g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	g_UserGenerator.GetSkeletonCap().StartTracking(user);
	g_nPlayer = user;
	return TRUE;

}
void FindPlayer()
{
	if (g_nPlayer != 0)
	{
		return;
	}
	XnUserID aUsers[20];
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}
void LostPlayer()
{
	g_nPlayer = 0;
	FindPlayer();

}
void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	if (!g_bCalibrated) // check on player0 is enough
	{
		printf("Look for pose\n");
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
		return;
	}

	AssignPlayer(user);
// 	if (g_nPlayer == 0)
// 	{
// 		printf("Assigned user\n");
// 		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
// 		g_UserGenerator.GetSkeletonCap().StartTracking(user);
// 		g_nPlayer = user;
// 	}
}


void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("Lost user %d\n", user);
	if (g_nPlayer == user)
	{
		LostPlayer();
	}
}
void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt)
{
	printf("Found pose \"%s\" for user %d\n", strPose, user);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt)
{
	printf("Calibration started\n");
}

void XN_CALLBACK_TYPE CalibrationEnded(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* cxt)
{
	printf("Calibration done [%d] %ssuccessfully\n", user, bSuccess?"":"un");
	if (bSuccess)
	{
		if (!g_bCalibrated)
		{
			g_UserGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			g_nPlayer = user;
			g_UserGenerator.GetSkeletonCap().StartTracking(user);
			g_bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
}

void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt)
{
	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!g_bCalibrated)
		{
			g_UserGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			g_nPlayer = user;
			g_UserGenerator.GetSkeletonCap().StartTracking(user);
			g_bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
}


int main()
{

 fann_type *calc_out;
    fann_type input[2];
    struct fann *ann = fann_create_from_file("hand_float.net");

	XnStatus nRetVal = XN_STATUS_OK;
	xn::Context context;

	nRetVal = context.Init();
	if (nRetVal)
    {
        printf("Error1");
    }
	nRetVal = g_UserGenerator.Create(context);
	if (nRetVal)
    {
        printf("Error1");
    }

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	if (nRetVal)
    {
        printf("Error1");
    }
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal)
    {
        printf("Error1");
    }

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	{
		printf("User generator doesn't support either skeleton or pose detection.\n");
		return XN_STATUS_ERROR;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	if (nRetVal)
    {
        printf("Error1");
    }

	XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	g_UserGenerator.RegisterUserCallbacks(NewUser, LostUser, NULL, hUserCBs);
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(CalibrationStarted, NULL, hCalibrationStartCB);
	if (nRetVal)
    {
        printf("Error1");
    }
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, NULL, hCalibrationCompleteCB);
	if (nRetVal)
    {
        printf("Error1");
    }
	nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, NULL, hPoseCBs);
	if (nRetVal)
    {
        printf("Error1");
    }


nRetVal = context.StartGeneratingAll();

	while(TRUE)
{
	nRetVal =context.WaitAndUpdateAll();
	if (nRetVal)
    {
        printf("Error1");
    }
   XnUserID aUsers[15];
   XnUInt16 nUsers=15;
   g_UserGenerator.GetUsers(aUsers,nUsers);
	for(int i=0;i<nUsers;++i)
	{
		if(g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			XnSkeletonJointPosition Head,Hand;
			g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i],XN_SKEL_HEAD,Head);
		//printf("head%d: (%f,%f,%f) [%f]\n", aUsers[i],Head.position.X, Head.position.Y, Head.position.Z,Head.fConfidence);

g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i],XN_SKEL_LEFT_HAND,Hand);
		printf("hand%d: (%f,%f,%f) [%f]\n", aUsers[i],1/Hand.position.X,1/Hand.position.Y, 1/Hand.position.Z,1/Hand.fConfidence);
		input[0]=1/Hand.position.X;
		input[1]=1/Hand.position.Y;
		input[2]=1/Hand.position.Z;
calc_out=fann_run(ann,input);
if(calc_out[0]>0.900000)
		{printf("HAND POSE DETECTED");}
else
	printf("%f",calc_out[0]);
		}
		
	
	}
 }

		
	
}
	

