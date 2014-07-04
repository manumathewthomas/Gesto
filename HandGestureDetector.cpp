#include <stdio.h>
#include "ni/XnCppWrapper.h"
#include "nite/XnVSessionManager.h"
#include "nite/XnVMultiProcessFlowClient.h"
#include "nite/XnVWaveDetector.h"
#include "nite/XnVPushDetector.h"
#include "nite/XnVCircleDetector.h"
#include "nite/XnVHandPointContext.h"
#include "nite/XnVSwipeDetector.h"
#include "nite/XnVMultipleHands.h"
#include "floatfann.h"
#include "fann.h"


#include <linux/input.h>
#include <linux/uinput.h>





#define SAMPLE_XML_FILE "/root/Kinect/nite/Data/Sample-Tracking.xml"




static int uinp_fd = -1;
struct uinput_user_dev uinp; // uInput device structure
struct input_event event; // Input device structure

XnBool g_bQuit = false;

int setup_uinput_device()
{
    int i=0;
    uinp_fd = open("/dev/uinput", O_WRONLY | O_NDELAY);
    if (!uinp_fd)
    {
        printf("Unable to open /dev/uinput\n");
        return -1;
    }
    memset(&uinp,0,sizeof(uinp)); // Intialize the uInput device to NULL
    strncpy(uinp.name, "Kinect Mouse", UINPUT_MAX_NAME_SIZE);
    uinp.id.version = 4;
    uinp.id.bustype = BUS_USB;
    ioctl(uinp_fd, UI_SET_EVBIT, EV_KEY);
    ioctl(uinp_fd, UI_SET_EVBIT, EV_REL);
    ioctl(uinp_fd, UI_SET_RELBIT, REL_X);
    ioctl(uinp_fd, UI_SET_RELBIT, REL_Y);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_MOUSE);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_TOUCH);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_MOUSE);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_LEFT);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_MIDDLE);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_RIGHT);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_FORWARD);
    ioctl(uinp_fd, UI_SET_KEYBIT, BTN_BACK);
    /* Create input device into input sub-system */
    write(uinp_fd, &uinp, sizeof(uinp));
    if (ioctl(uinp_fd, UI_DEV_CREATE))
    {
        printf("Unable to create UINPUT device.");
        return -1;
    }
    return 1;
}
void move_cursor(int x, int y )
{
    memset(&event, 0, sizeof(event));
    gettimeofday(&event.time, NULL);
    event.type = EV_REL;
    event.code = REL_X;
    event.value = x;
    write(uinp_fd, &event, sizeof(event));
    event.type = EV_REL;
    event.code = REL_Y;
    event.value = y;
    write(uinp_fd, &event, sizeof(event));
    event.type = EV_SYN;
    event.code = SYN_REPORT;
    event.value = 0;
    write(uinp_fd, &event, sizeof(event));
}

void send_click_events( )
{
printf("CLICK DETECTED");
// Move pointer to (0,0) location
memset(&event, 0, sizeof(event));
gettimeofday(&event.time, NULL);
event.type = EV_REL;
event.code = REL_X;
event.value = 100;
write(uinp_fd, &event, sizeof(event));
event.type = EV_REL;
event.code = REL_Y;
event.value = 100;
write(uinp_fd, &event, sizeof(event));
event.type = EV_SYN;
event.code = SYN_REPORT;
event.value = 0;
write(uinp_fd, &event, sizeof(event));
// Report BUTTON CLICK - PRESS event
memset(&event, 0, sizeof(event));
gettimeofday(&event.time, NULL);
event.type = EV_KEY;
event.code = BTN_LEFT;
event.value = 1;
write(uinp_fd, &event, sizeof(event));
event.type = EV_SYN;
event.code = SYN_REPORT;
event.value = 0;
write(uinp_fd, &event, sizeof(event));
// Report BUTTON CLICK - RELEASE event
memset(&event, 0, sizeof(event));
gettimeofday(&event.time, NULL);
event.type = EV_KEY;
event.code = BTN_LEFT;
event.value = 0;
write(uinp_fd, &event, sizeof(event));
event.type = EV_SYN;
event.code = SYN_REPORT;
event.value = 0;
write(uinp_fd, &event, sizeof(event));
}



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
//



void XN_CALLBACK_TYPE SessionProgress(const XnChar* strFocus,const XnPoint3D& ptFocusPoint,XnFloat fProgress,void* UserCxt)
{
    printf("Session progress (%6.2f,%6.2f,%6.2f) -%6.2f [%s] \n",ptFocusPoint.X,ptFocusPoint.Y,ptFocusPoint.Z,fProgress,strFocus);
}

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocusPoint,void* UserCxt)
{
    printf("Session started .Please Wave (%6.2f,%6.2f,%6.2f)...\n",ptFocusPoint.X,ptFocusPoint.Y,ptFocusPoint.Z);
}

void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
    printf("Session ended.Please perform focus gesture to start session\n");
}

void XN_CALLBACK_TYPE OnWaveCB(void* cxt)
{
    char* prog[3];
    prog[0] = "firefox";
    prog[1] = "http://www.yahoo.com";
    prog[2] = '\0';

    execvp("firefox", prog);
}

void XN_CALLBACK_TYPE OnPushCB(XnFloat fVelocity, XnFloat fAngle,void* cxt)
{

}



void XN_CALLBACK_TYPE CircleCB(XnFloat fTimes, XnBool bConfident, const XnVCircle* pCircle, void* pUserCxt)
{

       
}

void XN_CALLBACK_TYPE OnPointUpdate(const XnVHandPointContext* pContext,void* cxt)
{
       move_cursor((int)(pContext->ptPosition.X/8),(int) -(pContext->ptPosition.Y/8));
}

int main(int argc,char** argv)
{
fann_type *calc_out;
    fann_type input[2];
    struct fann *ann = fann_create_from_file("click_float.net");

    XnStatus nRetVal =XN_STATUS_OK;

    xn::Context context;
    nRetVal =context.Init();
    if (nRetVal)
    {
        printf("Error1");
    }
    context.SetGlobalMirror(true);
    XnStatus rc = context.InitFromXmlFile(SAMPLE_XML_FILE);
    if (rc != XN_STATUS_OK)
    {
            printf("Couldn't initialize: %s\n", xnGetStatusString(rc));
            return 1;
    }
    XnVSessionManager* sessionManager = new XnVSessionManager();
    nRetVal= ((XnVSessionManager*)sessionManager)->Initialize(&context, "Click", "RaiseHand");
 setup_uinput_device();


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




    if(nRetVal)
        printf("eroor");
    nRetVal = context.StartGeneratingAll();
    if(nRetVal)
    printf("error 4");
    sessionManager->RegisterSession(NULL,&SessionStart,&SessionEnd);

    XnVWaveDetector wc;
    XnVPushDetector ps;
    XnVCircleDetector cs;

    wc.RegisterWave(NULL,OnWaveCB);
    wc.RegisterPointUpdate(NULL,OnPointUpdate);
    ps.RegisterPush(NULL,OnPushCB);
    cs.RegisterCircle(NULL,CircleCB);

    sessionManager->AddListener(&wc);
    sessionManager->AddListener(&ps);
    sessionManager->AddListener(&cs);

    printf("Please perform focus gesture to start session\n");
    printf("Hit any key to exit\n");
     
    while (TRUE)
    {
        nRetVal =context.WaitAndUpdateAll();
        if(nRetVal)
            printf("error 5");
    sessionManager->Update(&context);
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

g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i],XN_SKEL_RIGHT_HAND,Hand);
		printf("hand%d: (%f,%f,%f) [%f]\n", aUsers[i],1/Hand.position.X,1/Hand.position.Y, 1/Hand.position.Z,1/Hand.fConfidence);
		input[0]=1/Hand.position.X;
		input[1]=1/Hand.position.Y;
		input[2]=1/Hand.position.Z;
calc_out=fann_run(ann,input);
if(calc_out[0]>0.900000)
		{send_click_events();}
else
	printf("%f",calc_out[0]);
		}
		
	
	}
    }

 context.Shutdown();
 ioctl(uinp_fd, UI_DEV_DESTROY);
 close(uinp_fd);
 return 0;

}
