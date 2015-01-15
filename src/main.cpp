//          _____                     _____                     _____                     _____          
//         /\    \                   /\    \                   /\    \                   /\    \         
//        /::\____\                 /::\    \                 /::\    \                 /::\    \        
//       /::::|   |                /::::\    \                \:::\    \               /::::\    \       
//      /:::::|   |               /::::::\    \                \:::\    \             /::::::\    \      
//     /::::::|   |              /:::/\:::\    \                \:::\    \           /:::/\:::\    \     
//    /:::/|::|   |             /:::/__\:::\    \                \:::\    \         /:::/__\:::\    \    
//   /:::/ |::|   |            /::::\   \:::\    \                \:::\    \       /::::\   \:::\    \   
//  /:::/  |::|___|______     /::::::\   \:::\    \                \:::\    \     /::::::\   \:::\    \  
// /:::/   |::::::::\    \   /:::/\:::\   \:::\    \                \:::\    \   /:::/\:::\   \:::\    \ 
///:::/    |:::::::::\____\ /:::/  \:::\   \:::\____\ _______________\:::\____\ /:::/__\:::\   \:::\____\
//\::/    / ~~~~~/:::/    / \::/    \:::\  /:::/    / \::::::::::::::::::/    / \:::\   \:::\   \::/    /
// \/____/      /:::/    /   \/____/ \:::\/:::/    /   \::::::::::::::::/____/   \:::\   \:::\   \/____/ 
//             /:::/    /             \::::::/    /     \:::\~~~~\~~~~~~          \:::\   \:::\    \     
//            /:::/    /               \::::/    /       \:::\    \                \:::\   \:::\____\    
//           /:::/    /                /:::/    /         \:::\    \                \:::\   \::/    /    
//          /:::/    /                /:::/    /           \:::\    \                \:::\   \/____/     
//         /:::/    /                /:::/    /             \:::\    \                \:::\    \         
//        /:::/    /                /:::/    /               \:::\____\                \:::\____\        
//        \::/    /                 \::/    /                 \::/    /                 \::/    /        
//         \/____/                   \/____/                   \/____/                   \/____/         
//                                                                                                   


/////////////////////////////////////////////////////////////////////////////
// 3D Maze
// Laura Byrnes-Blanco and Brian Wright
// University of South Florida
// EML4593 Haptics
// Dr. Kyle Reed
// Updated April 28, 2014
//////////////////////////////////////////////////////////////////////////////
//c  C++ HEADERS                                     

#include <QHHeadersWin32.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <fstream>  // ofstream
#include <io.h>
#include <iomanip>  // setw
#include <iostream> // cout, endl
#include <sstream>
#include <string.h>
using namespace std;

//c  ****  VARIABLES  *******
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
HDstring strongHand = "Left";
HDstring weakHand = "Right";
HDstring fileName = "dataTEST.txt";
bool ignoreFileExists = true;
const int feedbackMethod = 1; // 0 = none, 1 = force, 2 = position, 3 = thermal, 4 = single vibrotactile
const double mirror=0;
const int impulseRate = 1; // Hertz
const int impulsePercentOfCycle = 25; // 0 to 100
const int impulseDistance = 25; // mm
const double kOn = 0.070; // spring const, N/mm
const double kOff = 0.035; // spring const, N/mm
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Experimental Demo
//////////////////////////////////////////////////////////////////////////////


int dataStage = -5; // -5 = pretrial, 0 = start cell, 1 = recording, 2+ = finish line
int trialTimer = 0;
int signTimer = 1;
int impulseTimer = 0;
int impulsePeriod = (1000/(double)impulseRate);
int impulseOnPeriod = impulsePeriod*((double)impulsePercentOfCycle/100);
int impulseOffPeriod = impulsePeriod - impulseOnPeriod;
int minDTF = 36;
int word = -1;  // current distance to finish (DTF)
int baddata = 0;

class DataTransportClass//This class carried data into the ServoLoop thread
{
public:
	TriMesh* Model;//Trimesh pointer to hold mesh data
	Sphere* cursorSphere;//Sphere pointer. for the Sphere which replaces the cursor in this demo
	Cylinder* forceArrow;//The bar on the Sphere showing the magnitude and direction of force generated
    Cone* forceArrowTip;// The bar tip that points in the force direction.
	Cursor* deviceCursor;//Pointer to hold the cursor data
	Cursor* deviceCursor2;//Pointer to hold the cursor data
    Text* descriptionText;
};

//c  OPEN FILE TO RECORD DATA

hduMatrix WorldToDevice;//This matrix contains the World Space to DeviceSpace Transformation
hduVector3Dd forceVec;//This variable contains the force vector.

void GraphicsCallback(void);//Graphics callback routine
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata);//Servo loop callback
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata);//Servo Loop callback
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata);//Servo Loop callback
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius);//This function computer the force beween the Model and the particle based on the positions
std::ofstream myFile;
//c  *******************************************************************

//c  MAIN ROUTINE
int WINAPI WinMain(	HINSTANCE	hInstance,				// Instance
			HINSTANCE	hPrevInstance,				// Previous Instance
			LPSTR		lpCmdLine,				// Command Line Parameters
			int		nCmdShow)				// Window Show State
{

//	interfacekit_simple();
//	return 0;

//c  CHECK TO SEE IF FILE ALREADY EXISTS BEFORE OPENING
	if (ignoreFileExists == false) {
		if(_access(fileName, 0) != -1)
		{
			MessageBox(NULL, L"Data File Already Exists", L"Error!", MB_OK);
			return 0; // END DEBUG
		}
	}
  myFile.open (fileName, std::ofstream::out);

//c  INITIALIZE HAPTIC ENVIRONMENT
	QHWin32* DisplayObject = new QHWin32;//create a display window
	DataTransportClass dataObject;//Initialize an Object to transport data into the servoloop callback
	DisplayObject->hapticWindow(true);
    DisplayObject->setWindowTitle("Front View\0");

	QHWin32* DisplayObject2 = new QHWin32;//create a display window
    DisplayObject2->hapticWindow(false);//Disable haptics in this window
    DisplayObject2->setWindowTitle("Right View\0");//Set the title of this window as "Left View"

    QHWin32* DisplayObject3 = new QHWin32;//create a display window
    DisplayObject3->hapticWindow(false);//Disable Haptics in this window
    DisplayObject3->setWindowTitle("Top View\0");//Set the title of this window as "Top View"

//c  INITIALIZE BOTH OMNIS
	DeviceSpace* OmniSpace = new DeviceSpace(strongHand);//Find the default Phantom Device
	DeviceSpace* OmniSpace2 = new DeviceSpace(weakHand);//Find the other Phantom Device
    DisplayObject->setName("GRACEFUL AMAZING");//Give the window a title
//c  PLACE BOTH OMNIS IN SCENE
    DisplayObject->tell(OmniSpace);//Tell quickHaptics about the device space object    
    DisplayObject->tell(OmniSpace2);//Tell quickHaptics about the device space object    

    dataObject.cursorSphere = new Sphere(3,15);//Initialise a Sphere
	dataObject.cursorSphere->setName("cursorSphere");//Give it a name
    dataObject.cursorSphere->setShapeColor(0.8,0.2,0.2);//Give it a color
    dataObject.cursorSphere->setHapticVisibility(false);//Make the Sphere haptically invisible. this sphere replaces the cursor hence it must be haptically invisible or the proxy will keep colliding with the sphere
    DisplayObject->tell(dataObject.cursorSphere);//Tell QuickHaptics
    
    dataObject.forceArrow = new Cylinder(0.75,1,15);//Initialise a cylinder
    dataObject.forceArrow->setShapeColor(0.2,0.7,0.2);//Give it a color
    dataObject.forceArrow->setHapticVisibility(false);//Make it haptictically invisible
    dataObject.forceArrow->setName("forceArrow");//Give it a name
    DisplayObject->tell(dataObject.forceArrow);//tell Quickhaptics
        
    dataObject.forceArrowTip = new Cone(2,4,15);//Initialise a cone
    dataObject.forceArrowTip->setShapeColor(1.0,0.0,0.0);//Give it a color
    dataObject.forceArrowTip->setHapticVisibility(false);//Make it haptictically invisible
    dataObject.forceArrowTip->setName("forceArrowTip");//Give it a name
    DisplayObject->tell(dataObject.forceArrowTip);//tell Quickhaptics

    dataObject.Model = new TriMesh("Models/Test2.3DS");//Load a Skull  Model for the Mesh
    dataObject.Model->setName("Maze");//Give it a name
    dataObject.Model->setHapticVisibility(true);
    dataObject.Model->setGraphicVisibility(true);
	dataObject.Model->setShapeColor(0.5,0.5,0.5);//Make to color of the skull purple
//	dataObject.Model->setTexture("Models/cow.jpg");
	dataObject.Model->setScale( 1.0 ); // make the skull smaller, about the same size as the sphere
	dataObject.Model->setTouchableFace("front");
	dataObject.Model->setUnDraggable();
    DisplayObject->tell(dataObject.Model);//Tell QuickHaptics about it.

	Box* SimpleBox = new Box(140, 60, 100, 70, 30, -50); // DIM 140 60 100, LOC 70 -10 -50
	SimpleBox->setName("WOO");
	SimpleBox->setUnDraggable();
//    SimpleBox->setTexture("models/cow.jpg");
	SimpleBox->setTouchableFace("back");
	SimpleBox->setHapticVisibility(true);
	SimpleBox->setGraphicVisibility(false);
//	SimpleBox->setPopthrough();
	DisplayObject->tell(SimpleBox);

    dataObject.deviceCursor= new Cursor();//Get a new cursor
    dataObject.deviceCursor->setName("devCursor");//Give it a name
    dataObject.deviceCursor->setCursorGraphicallyVisible(false);//Make it graphically invisible
    DisplayObject->tell(dataObject.deviceCursor);//Tell Quickhaptics about it.

    dataObject.deviceCursor2 = new Cursor();//Get a new cursor
    dataObject.deviceCursor2->setName("devCursor2");//Give it a name
    dataObject.deviceCursor2->setCursorGraphicallyVisible(true);//Make it graphically invisible
    DisplayObject->tell(dataObject.deviceCursor2);//Tell Quickhaptics about it.

    dataObject.descriptionText = new Text(20.0,"W",0.4,0.9);
    dataObject.descriptionText->setName("TitleBox");
    dataObject.descriptionText->setShapeColor(0.7,0.0,0.4);
    DisplayObject->tell(dataObject.descriptionText);

    Text* descriptionText2 = new Text(20.0, "0 0 0 - 0", 0.4,0.85);
    descriptionText2->setName("TextBox");
    descriptionText2->setShapeColor(1.0,0.8,0.9);
    DisplayObject->tell(descriptionText2);
    Text* descriptionText3 = new Text(14.0, "000.0 ms", 0.4, 0.80);
	descriptionText3->setName("TimerBox");
    descriptionText3->setShapeColor(1.0,0.8,0.9);
    DisplayObject->tell(descriptionText3);
	Text* descriptionText4 = new Text(14.0, "Under Construction", 0.4,0.75);
    descriptionText4->setName("CommentBox");
    descriptionText4->setShapeColor(1.0,0.2,0.0);
    DisplayObject->tell(descriptionText4);

    DisplayObject->preDrawCallback(GraphicsCallback);//Register the graphics callback
    OmniSpace->startServoLoopCallback(startEffectCB, computeForceCB, stopEffectCB,&dataObject);//Register the servoloop callback

	//
	// Change the default camera, first set the Default Camera, 
	// then read back the fov, eye point etc.
	//

    float FOV1, FOV2, FOV3;
    float NearPlane1, NearPlane2, NearPlane3;
    float FarPlane1, FarPlane2, FarPlane3;
    hduVector3Dd Eye1, Eye2, Eye3;
    hduVector3Dd LookAt1, LookAt2, LookAt3;
    hduVector3Dd UpVector1, UpVector2, UpVector3;
  
    DisplayObject->setDefaultCamera();
    DisplayObject->getCamera(&FOV1,&NearPlane1,&FarPlane1,&Eye1,&LookAt1,&UpVector1);
    
    FOV3 = FOV2 = FOV1;
    NearPlane3 = NearPlane2 = NearPlane1;
    FarPlane3 = FarPlane2 = FarPlane1;
    Eye3 = Eye2 = Eye1;
    LookAt3 = LookAt2 = LookAt1;
    UpVector2 = UpVector1;
	UpVector3 = UpVector1;
    Eye2.set(Eye1[2]+50,LookAt2[1]-30,LookAt2[2]);
    Eye3.set(LookAt2[0],Eye1[2],LookAt2[2]);
    UpVector3.set(0.0,0.0,-1.0);
    LookAt2[0]=LookAt2[0]-50;
    DisplayObject2->setCamera(FOV2, NearPlane2-70, FarPlane2+70, Eye2, LookAt2, UpVector2);
    DisplayObject3->setCamera(FOV3, NearPlane3, FarPlane3, Eye3, LookAt3, UpVector3);
  
    dataObject.Model->setHapticVisibility(true);
    dataObject.Model->setGraphicVisibility(true);
	SimpleBox->setHapticVisibility(true);
	SimpleBox->setGraphicVisibility(false);

//c  A-MAZE-ING ROUTINE (MAIN)
	qhStart();

//c  MAIN ROUTINE ENDS, CLOSE FILE
	myFile.close();
	MessageBox(NULL, L"Trial Concluded!", L"Success", MB_OK);
}

//
// The Graphics Callback runs in the application "client thread" (qhStart) and sets the transformations
// for the Red Sphere and Green Line of the Cursor. Also, this callback sets the WorldToDevice matrix
// for use in the ServoLoopCallback.
//
void GraphicsCallback(void)
{
	QHWin32* localDisplayObject = QHWin32::searchWindow("GRACEFUL AMAZING");//Get a Pointer to the display object
    Cursor* localDeviceCursor = Cursor::searchCursor("devCursor");//Get a pointer to the cursor
    Cylinder* localForceArrow = Cylinder::searchCylinder("forceArrow");//get a pointer to the cylinder
    Cone* localForceArrowTip = Cone::searchCone("forceArrowTip");//get a pointer to the cylinder
	Sphere* localCursorSphere = Sphere::searchSphere("cursorSphere");//get a pointer top the Sphere
	TriMesh* localMazeModel = TriMesh::searchTriMesh("Maze");
	if( localDisplayObject == NULL || localDeviceCursor == NULL || localForceArrow == NULL || localCursorSphere == NULL)
		return;

	hduVector3Dd localCursorPosition1;
	hduVector3Dd localCursorPosition2;
    Cursor* localDeviceCursor1 = Cursor::searchCursor("devCursor");//Get a pointer to the cursor
    Cursor* localDeviceCursor2 = Cursor::searchCursor("devCursor2");//Get a pointer to the cursor
    localCursorPosition1 = localDeviceCursor1->getPosition();//Get the local cursor position in World Space
    localCursorPosition2 = localDeviceCursor2->getPosition();//Get the local cursor position in World Space

	hduVector3Dd CPosition;
	hduVector3Dd C2Position;
	CPosition = localDeviceCursor1->getPosition();//Get the current position of the haptic device
	C2Position = localDeviceCursor2->getPosition();//Get the current position of the haptic device

	int PosX = C2Position[0]/20;
	int PosY = C2Position[1]/20;
	int PosZ = C2Position[2]/20+5;

	double FinePosX1 = CPosition[0]; // TODO WHY IS THIS RANGE DIFFERENT?!
	double FinePosY1 = CPosition[1];
	double FinePosZ1 = CPosition[2];

	double FinePosX2 = C2Position[0];
	double FinePosY2 = C2Position[1];
	double FinePosZ2 = C2Position[2];

	if ((FinePosX1==FinePosX2)&&(FinePosY1==FinePosY2)) { baddata = 1; } else { baddata = 0; }

	if ((word < minDTF)&&(word!=-1)&&(dataStage==1)&&(baddata==0)) minDTF=word;

	if (baddata==0)
	{
	myFile << std::setw(10) << trialTimer << " " << std::setw(10) << word;
	myFile << " " << std::setw(10) << minDTF << " " << std::setw(10) << FinePosX1 << " " << std::setw(10) << FinePosY1 << " " << std::setw(10) << FinePosZ1;
	myFile << " " << std::setw(10) << FinePosX2 << " " << std::setw(10) << FinePosY2 << " " << std::setw(10) << FinePosZ2 ;
	myFile << std::endl;
	}

	const int dist[7][3][5] = { { { 14, 13, 12, 11, 10}, { -1, -1, -1, -1,  9}, { 36, 35, 34, -1,  8} },
							    { { -1, -1, 13, -1, -1}, { -1, -1, -1, -1, -1}, { -1, -1, 33, -1,  7} },
							    { { 32, -1, 14, 15, 16}, { 33, -1, -1, -1, -1}, { 34, -1, 32, -1,  6} },
							    { { 31, -1, -1, -1, 17}, { -1, -1, -1, -1, -1}, { -1, -1, 31, -1,  5} },
							    { { 30, -1, 20, 19, 18}, { 29, -1, -1, -1, -1}, { 28, 29, 30, -1,  4} },
							    { { -1, -1, 21, -1, -1}, { -1, -1, -1, -1, -1}, { 27, -1, -1, -1,  3} },
							    { { 24, 23, 22, -1,  0}, { 25, -1, -1, -1,  1}, { 26, 27, 28, -1,  2} } };

	if ((PosX > -1) & (PosX < 7) & (PosY > -1) & (PosY < 3) & (PosZ > -1) & (PosZ < 5))
	{
		word = dist[PosX][PosY][PosZ];
	}
	else
	{
		word = -1;
	}
	std::stringstream strs;
	std::stringstream stri;
	strs << PosX << " " << PosY << " " << PosZ << " - " << word;
	stri << 0;
	std::string temp_str = strs.str();
	std::string timer_str = stri.str();
	char* pchar = (char*) temp_str.c_str();
	char* tchar = (char*) timer_str.c_str();

	Text* TextPointer = Text::searchText("TextBox");
	Text* TimerPointer = Text::searchText("TimerBox");
	TextPointer->update(pchar,0.45,0.3);	
	TimerPointer->update(tchar,0.45,0.2);

	hduMatrix CylinderTransform;//Transformation for the Cylinder. This transform makes it point toward the Model
	hduVector3Dd localCursorPosition;
	hduVector3Dd DirectionVecX;
	hduVector3Dd PointOnPlane;
	hduVector3Dd DirectionVecY;
	hduVector3Dd DirectionVecZ;

	//Compute the world to device transform
    WorldToDevice = localDisplayObject->getWorldToDeviceTransform();

	// Set transform for Red Sphere
    localCursorPosition = localDeviceCursor->getPosition();//Get the local cursor position in World Space
	
	hduVector3Dd localCursorSpherePos = localCursorSphere->getTranslation();
	localCursorSphere->setTranslation(-localCursorSpherePos);
	localCursorSphere->setTranslation(localCursorPosition);//Set the position of the Sphere the same as the cursor
    
	////////////////////////////////////////////////////////////////////////////////////////////
	//Code to calculate the transform of the green cylinder to point along the force direction
	////////////////////////////////////////////////////////////////////////////////////////////
	
	hduMatrix DeviceToWorld = WorldToDevice.getInverse();
	HDdouble ForceMagnitude = forceVec.magnitude();
	DeviceToWorld[3][0] = 0.0;
	DeviceToWorld[3][1] = 0.0;	
	DeviceToWorld[3][2] = 0.0;
	DirectionVecX = forceVec * DeviceToWorld;
    DirectionVecX.normalize();
    PointOnPlane.set(0.0,0.0,(DirectionVecX[0]*localCursorPosition[0] + DirectionVecX[1]*localCursorPosition[1] + DirectionVecX[2]*localCursorPosition[2])/DirectionVecX[2]);
    DirectionVecY = PointOnPlane  - localCursorPosition;
    DirectionVecY.normalize();

	/////////////////////////////////////////////
}


/***************************************************************************************
 Servo loop thread callback.  Computes a force effect. This callback defines the motion
 of the purple skull and calculates the force based on the "real-time" Proxy position
 in Device space
****************************************************************************************/
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata)
{

	hduVector3Dd localCursorPosition1;
	hduVector3Dd localCursorPosition2;
    Cursor* localDeviceCursor1 = Cursor::searchCursor("devCursor");//Get a pointer to the cursor
    Cursor* localDeviceCursor2 = Cursor::searchCursor("devCursor2");//Get a pointer to the cursor
    localCursorPosition1 = localDeviceCursor1->getPosition();//Get the local cursor position in World Space
    localCursorPosition2 = localDeviceCursor2->getPosition();//Get the local cursor position in World Space
	TriMesh* damnMaze = TriMesh::searchTriMesh("Maze");
	Box* wooBox = Box::searchBox("WOO");
	// TODO Check for left-hand inconsistency
	DeviceSpace* SpacePointer = DeviceSpace::searchSpace((char*)strongHand);//Search for the haptic device and return a pointer to it.
	DeviceSpace* SpacePointer2 = DeviceSpace::searchSpace((char*)weakHand);//Search for the haptic device and return a pointer to it.
	
	hduVector3Dd CPosition;
	hduVector3Dd C2Position;
	CPosition = localDeviceCursor1->getPosition();//Get the current position of the haptic device
	C2Position = localDeviceCursor2->getPosition();//Get the current position of the haptic device
	
	hduVector3Dd Force = SpacePointer->getForce();//Get the current force being exerted by the haptic device.
	hduVector3Dd Tracker;
	hduVector3Dd HomingPigeon;

	const int hint[7][3][5] = { { {  2,  2,  2,  2,  8}, { -1, -1, -1, -1,  8}, {  2,  2, 32, -1, 32} },
							    { { -1, -1, 16, -1, -1}, { -1, -1, -1, -1, -1}, { -1, -1, 32, -1, 32} },
							    { { 32, -1, 16,  1,  1}, {  4, -1, -1, -1, -1}, {  4, -1, 32, -1, 32} },
							    { { 32, -1, -1, -1, 16}, { -1, -1, -1, -1, -1}, { -1, -1, 32, -1, 32} },
							    { {  8, -1,  2,  2, 16}, {  8, -1, -1, -1, -1}, { 32,  1,  1, -1, 32} },
							    { { -1, -1, 16, -1, -1}, { -1, -1, -1, -1, -1}, { 32, -1, -1, -1, 32} },
							    { {  2,  2, 16, -1,  0}, {  4, -1, -1, -1,  0}, {  4,  1,  1, -1,  4} } };
/* 128 = START    64 = FINISH    32 = X+ (RIGHT)    16 = X-     8 = Y+ (FRONT)     4 = Y-     2 = Z+ (DOWN)     1 = Z-  */

	int PosX = C2Position[0]/20;
	int PosY = C2Position[1]/20;
	int PosZ = C2Position[2]/20+5;
	
	double FinePosX1 = CPosition[0]; // TODO WHY IS THIS RANGE DIFFERENT?!
	double FinePosY1 = CPosition[1];
	double FinePosZ1 = CPosition[2];

	double FinePosX2 = C2Position[0];
	double FinePosY2 = C2Position[1];
	double FinePosZ2 = C2Position[2];

	if ((FinePosX1==FinePosX2)&&(FinePosY1==FinePosY2)) { baddata = 1; } else { baddata = 0; }
	
//c  DRAG OMNI INTO BOX
	HomingPigeon[0] = 0;
	HomingPigeon[1] = 0;
	HomingPigeon[2] = 0;

			if ((dataStage == -5) && ((FinePosX2<5) || (FinePosX2>15) || (FinePosY2<45) || (FinePosY2>55) || (FinePosZ2<-95) || (FinePosZ2>-85)))
			{
				damnMaze->setShapeColor(0.5, 0, 0);
				damnMaze->setHapticVisibility(false);
				HomingPigeon[0] = 0.004*(10-FinePosX2);
				HomingPigeon[1] = 0.004*(50-FinePosY2);
				HomingPigeon[2] = 0.004*(-90-FinePosZ2);
			}
			else if ((dataStage == -5)&&(baddata==0))
			{
				damnMaze->setHapticVisibility(true);
				dataStage++;
				damnMaze->setShapeColor(0.5, 0.5, 0);
			}
			else if ((dataStage == 0)&&((PosX != 0) || (PosY != 2) || (PosZ != 0))&&(baddata==0))
			{
					dataStage = 1;
					damnMaze->setShapeColor(0, 0, 0);

			}
			else if ((dataStage == 1)&&(baddata==0))
			{
				trialTimer++;
				if ((feedbackMethod==1)&&(baddata==0)){
					hdBeginFrame(hdGetCurrentDevice());
					int hintForce;
					if ((PosX > -1) & (PosX < 7) & (PosY > -1) & (PosY < 3) & (PosZ > -1) & (PosZ < 5))
					{
						hintForce = hint[PosX][PosY][PosZ];
					}
					else
					{
						hintForce = -1;
					}
					hduVector3Dd f(0,0.4,0);
					switch(hintForce){
						case 1:
							f[0] = 0;
							f[1] = 0  + 0.3;
							f[2] = -1.3;
							break;
						case 2:
							f[0] = 0;
							f[1] = 0  + 0.3;
							f[2] = 1.3;
							break;
						case 4:
							f[0] = 0;
							f[1] =-0.7  + 0.4;
							f[2] = 0;
						break;
						case 8:
							f[0] = 0;
							f[1] = 0.7  + 0.4;
							f[2] = 0;
							break;
						case 16:
							f[0] = -0.7 * (mirror-0.5)*(-2);
							f[1] = 0  + 0.4;
							f[2] = 0;
							break;
						case 32:
							f[0] = 0.7 * (mirror-0.5)*(-2);
							f[1] = 0  + 0.4;
							f[2] = 0;
							break;
						default:
							f[0] = 0;
							f[1] = 0  + 0.4;
							f[2] = 0;
					}
					hdSetDoublev(HD_CURRENT_FORCE, f);
					hdEndFrame(hdGetCurrentDevice());
				}
				else if ((feedbackMethod == 2)&&(baddata==0)){
					// TODOO
					hdBeginFrame(hdGetCurrentDevice());
					int hintForce;
					if ((PosX > -1) & (PosX < 7) & (PosY > -1) & (PosY < 3) & (PosZ > -1) & (PosZ < 5))
					{
						hintForce = hint[PosX][PosY][PosZ];
					}
					else
					{
						hintForce = -1;
					}
					impulseTimer++;
					hduVector3Dd X(-FinePosX1 + 70,-FinePosY1 + 10,-FinePosZ1 - 100);
					double ka = kOn;
					if (impulseTimer<impulseOnPeriod){
						switch(hintForce){
							case 1:
								X[2] = X[2]-impulseDistance;
								break;
							case 2:
								X[2] = X[2]+impulseDistance;
								break;
							case 4:
								X[1] = X[1]-impulseDistance;
								break;
							case 8:
								X[1] = X[1]+impulseDistance;
								break;
							case 16:
								X[0] = X[0]-impulseDistance;
								break;
							case 32:
								X[0] = X[0]+impulseDistance;
								break;
							default:
 								X[0] = 0;
								X[1] = 0;
								X[2] = 0;
						}
					} else {
						ka = kOff;
						if (impulseTimer>=impulsePeriod){
							impulseTimer = 0;
						}
					}
					hduVector3Dd f(ka*X[0],ka*X[1]+0.4,ka*X[2]);
					hdSetDoublev(HD_CURRENT_FORCE, f);
					hdEndFrame(hdGetCurrentDevice());
				}
				if ((PosX == 6) && (PosY == 0) && (PosZ == 4) && (baddata==0))
				{
					dataStage = 2;
					damnMaze->setShapeColor(0, 1, 0);
					//damnMaze->setHapticVisibility(false);
					//wooBox->setHapticVisibility(false);
				}
			}


	//	SpacePointer2->setConstantForce(HomingPigeon,-2);
/*	hduVector3Dd DEBUG;
	DEBUG[0]=0;
	DEBUG[1]=0;
	DEBUG[2]=0;
	SpacePointer->setConstantForce(DEBUG,0);*/
	SpacePointer2->setConstantForce(HomingPigeon,1);
//	SpacePointer->setConstantForce(DEBUG,2);
//	SpacePointer2->setConstantForce(DEBUG,0);
		
//	myFile << " " << std::setw(10) << FinePosX2 << " " << std::setw(10) << FinePosY2 << " " << std::setw(10) << FinePosZ2 ;
//	myFile << std::setw(10) << i << " " << std::setw(10) << FinePosX1 << " " << std::setw(10) << FinePosY1 << " " << std::setw(10) << FinePosZ1;
//	myFile << std::endl;

	
//c   TRACK DTF  [DISTANCE TO FINISH]
	const int dist[7][3][5] = { { { 14, 13, 12, 11, 10}, { -1, -1, -1, -1,  9}, { 36, 35, 34, -1,  8} },
							    { { -1, -1, 13, -1, -1}, { -1, -1, -1, -1, -1}, { -1, -1, 33, -1,  7} },
							    { { 32, -1, 14, 15, 16}, { 33, -1, -1, -1, -1}, { 34, -1, 32, -1,  6} },
							    { { 31, -1, -1, -1, 17}, { -1, -1, -1, -1, -1}, { -1, -1, 31, -1,  5} },
							    { { 30, -1, 20, 19, 18}, { 29, -1, -1, -1, -1}, { 28, 29, 30, -1,  4} },
							    { { -1, -1, 21, -1, -1}, { -1, -1, -1, -1, -1}, { 27, -1, -1, -1,  3} },
							    { { 24, 23, 22, -1,  0}, { 25, -1, -1, -1,  1}, { 26, 27, 28, -1,  2} } };
	int word;
	if ((PosX > -1) & (PosX < 7) & (PosY > -1) & (PosY < 3) & (PosZ > -1) & (PosZ < 5))
	{
		word = dist[PosX][PosY][PosZ];
	}
	else
	{
		word = -1;
	}
	std::stringstream strs;
	std::stringstream stri;
	strs << PosX << " " << PosY << " " << PosZ << " - " << word;
	stri << 0;
	std::string temp_str = strs.str();
	std::string timer_str = stri.str();
	char* pchar = (char*) temp_str.c_str();
	char* tchar = (char*) timer_str.c_str();


	DataTransportClass *localdataObject = (DataTransportClass *) userdata;//Typecast the pointer passed in appropriately

	hduVector3Dd skullPositionDS;//Position of the skull (Moving sphere) in Device Space.
	hduVector3Dd proxyPosition;//Position of the proxy in device space
	HDdouble instRate = 0.0;
	HDdouble deltaT = 0.0;
	static float counter = 0.0;
	static int counter1 = 0;

    // Get the time delta since the last update.
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
    deltaT = 1.0 / instRate;
    counter+=deltaT;
	
//	hduVector3Dd ModelPos = localdataObject->Model->getTranslation();

    WorldToDevice.multVecMatrix(localdataObject->Model->getTranslation(),skullPositionDS);//Convert the position of the sphere from world space to device space

	hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPosition);//Get the position of the proxy in Device Coordinates (All HL commands in the servo loop callback fetch values in device coordinates)
    proxyPosition[0]=proxyPosition[0]-10;
    proxyPosition[1]=proxyPosition[1]+10;
    proxyPosition[2]=proxyPosition[2]+20;
	forceVec = forceField(proxyPosition, skullPositionDS, 40.0, 5.0);//Calculate the force
// TODO

	forceVec[0]=0.0*(localCursorPosition2[0]-localCursorPosition1[0]);
	forceVec[1]=0.0*(localCursorPosition2[1]-localCursorPosition1[1]);
	forceVec[2]=0.0*(localCursorPosition2[2]-localCursorPosition1[2]);

    counter1++;
    if(counter1>2000)//Make the force start after 2.0 seconds of program start. This is because the servo loop thread executes before the graphics thread. 
		//Hence global variables set in the graphics thread will not be valid for sometime in the begining og the program
    {
		force[0] = 0.1*HomingPigeon[0];
		force[1] = 0.1*HomingPigeon[1];
		force[2] = 0.1*HomingPigeon[2];
		if (((dataStage > 1) && (dataStage < 10)) || ((dataStage < 0)&&(dataStage > -5)))
		{
			signTimer++;
			if (signTimer<=40)
			{
				force[1] = 4*(((double)signTimer/2)-(signTimer/2))-1;
   				force[1] = 2*force[1];
			}
			else if (signTimer>100)
			{
				signTimer = 0;
				dataStage++;
			}
		}

        counter1 = 2001;
	}
    else
    {
        force[0] = 0.0;
        force[1] = 0.0;
        force[2] = 0.0;
    }
}


/******************************************************************************
 Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
    DataTransportClass *localdataObject = (DataTransportClass *) userdata;
    printf("Custom effect started\n");
}


/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
    printf("Custom effect stopped\n");
}


/*******************************************************************************
 Given the position of the two charges in space,
 calculates the  force.
*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius)
{
    hduVector3Dd diffVec = Pos2 - Pos1 ;//Find the difference in position
    double dist = 0.0;
    hduVector3Dd forceVec(0,0,0);
	
    HDdouble nominalMaxContinuousForce;
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &nominalMaxContinuousForce);//Find the max continuous force that the device is capable of

    dist = diffVec.magnitude();

	if(dist < Radius*2.0) //Spring force when the model and cursor are within a 'sphere of influence'
    {
        diffVec.normalize();
        forceVec =  (Multiplier) * diffVec * dist /(4.0 * Radius * Radius);
        static int i=0;
    }
    else //Inverse square attraction
    {
        forceVec = Multiplier * diffVec/(dist*dist);
    }

    for(int i=0;i<3;i++)//Limit force calculated to Max continuous. This is very important because force values exceeding this value can damage the device motors.
    {
        if(forceVec[i]>nominalMaxContinuousForce)
            forceVec[i] = nominalMaxContinuousForce;

        if(forceVec[i]<-nominalMaxContinuousForce)
            forceVec[i] = -nominalMaxContinuousForce;
    }

	return forceVec;
}





