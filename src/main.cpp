/////////////////////////////////////////////////////////////////////////////
// 3D Maze
// Laura Byrnes-Blanco and Brian Wright
// University of South Florida
// EML4593 Haptics
// Dr. Kyle Reed
// Updated April 22, 2014
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Experimental Demo
//////////////////////////////////////////////////////////////////////////////

//////////////////
//     TODO     //
//////////////////////////////////////////////////////////////////////////////
//
// CODE IS IN THE PROCESS OF BEING REWRITTEN USING PSEUDOCODE TEMPLATE!
// SEARCH FOR "NEW BEGINNING".
//
//  Link program to external sensors/actuators
//    Phidgets
//      * Library linked
//      * Need to output to Peltier
//         - Op amp / power amp
//         - Wires
//         - Power source
//      * Holder for plates e.g. glove pockets
//
//    Arduino as an alternative?? (It looks like Serial In/Out can be used on Arduino from VS... somehow.)
//
//  Is gimbal position important?
//  ...Useful to change properties (lighting, friction) based on X/Y/Z angle?
//
//  Accurate Stopwatch (link to Servo callback)
//STOPWATCH HACK IN PLACE
//    Alternate: gradually fill progress bar (color change, finish line, etc)
//
//  Tweak Right Camera
//MOSTLY DONE (Better view possible?)
//
//  Implement Bimanual
//PARTIALLY COMPLETE
//
//  Hint Force e.g. solve automatically
//BASIC ROUTINE IMPLEMENTED
//    Center on final box
//    Center on normal area of box while in motion?
//    Link to Stylus Button 2 ?
//    Restore weightless force after hint force (marked "TODO")
//    Optimize for bimanual e.g. make left omni provide feedback to guide right
//
//  Stylus Button 1 to Open All Walls->Return to Start Point->Close Walls
//    Center on beginning box
//    Alternately, make button begin timer
//
//  Add data metrics
//DONE    Routine to write to raw file
//DONE    * Fine path tracing (20 divisions per block)
//    * Total number of backtracks
//    * Percentage of time in contact with walls
//    * Feedback vs no feedback
//    * Bimanual ??
//    * ??
//
//  Change project file names and folders to accurate titles (e.g. not SpongyCow)
//DONE
//
//  Fix erratic jerking/vibrations (worst at corners farthest from Omni)
//    -> ONLY ONE OMNI HAS VIBRATION PROBLEMS
//       - Possibly use problematic omni as the guidance omni and other as the navigating omni
//       - If other is used for guidance, do we need to have it also in the maze boundaries?
//       - Omni 2 can apply a left force, making the user push right to keep motionless,
//         but pushing right on both Omnis could keep the guidance once motionless while the right moves thru maze.
//    * Chamfers - make problem worse!
//    * Change virtual object location - sufficient (moved dir[1] negative 40)
//    * Enable popthru and inward force if out of bounds - untested
//    * Smaller outer box dimensions (overlap) - ineffective
//    * ??
//
//////////////////////////////////////////////////////////////////////////////


/* *************************************************** */
/* *************************************************** */
// FULL REWRITE UNTIL "NEW CODE ENDS" (NEW BEGINNING)
/* 3D MAZE PSEUDOCODE IS DENOTED BY //c AT THE BEGINNING OF EACH LINE */
/* *************************************************** */
/* *************************************************** */
//c  C++ HEADERS
//#include <phidget21.h>
#include <QHHeadersWin32.h>
//#include <iostream>
//#include <iomanip>
//#include <fstream>
//#include <sstream>
//#include <string.h>
//#include <math.h>

//c  **  VARIABLES  ****************************************************
//c  DETERMINE WHETHER RIGHT OR LEFT IS STRONG (DOMINANT HAND)
HDstring stronghand = "right";
HDstring weakhand = "left";
void MotionCallback(unsigned int ShapeID);

//c  *******************************************************************

//c  MAIN ROUTINE
int WINAPI WinMain(	HINSTANCE	hInstance,				// Instance
			HINSTANCE	hPrevInstance,				// Previous Instance
			LPSTR		lpCmdLine,				// Command Line Parameters
			int		nCmdShow)				// Window Show State
{

    

//    Cursor* OmniCursor = new Cursor;//Create a cursor
//    DisplayObject->tell(OmniCursor);//Tell QuickHaptics that a cursor exists

//c  INITIALIZE HAPTIC ENVIRONMENT
QHWin32* DisplayObject = new QHWin32;

//c  INITIALIZE STRONG OMNI
DeviceSpace* OmniStrong = new DeviceSpace(stronghand);
DeviceSpace* OmniWeak = new DeviceSpace(weakhand);

//c  PLACE STRONG OMNI IN SCENE
//DisplayObject->tell(OmniStrong);

//c TRAINING:
//c  CREATE A BOX FOR L-R
Box* SimpleBox = new Box(140, 20, 20, 0, 0, 0);
SimpleBox->setTouchableFace("back");
DisplayObject->tell(SimpleBox);
//c  DRAG OMNI INTO BOX
//c  LET USER EXPLORE
//c  CHANGE TO BOX Up-Down
//c  CHANGE TO BOX In-Out
//c  CLEAR SCENE
//c  INITIALIZE MAZE SCENE
//c LOOPBACK:
//c  DEACTIVATE WALLS
//c  FORCED MOVE TO STARTCELL
//c  ACTIVATE WALLS
//c  INITIALIZE WEAK OMNI
//DeviceSpace* OmniWeak = new DeviceSpace(weakhand);
//c  IF (METHOD==POSITION) ATTACH WEAK OMNI TO SCENE
////c Should the left be placed in the maze or do you think not linking it to the maze is best?
////c I was thinking it might be easy to keep it locked in the maze for the position guided sequence so we can use the voxils as a road map. But then we'd have to chance the code or have another file for the force driven guidance...
//c  INCREMENT FILE NUMBER -> (IF exists(n) THEN n++)
//c  OPEN FILE TO RECORD DATA
//c  START QUICKHAPTICS
qhStart();
SimpleBox->~Box();
qhStart();
//c  MAIN ROUTINE ENDS
}

//c  OnEvent ROUTINES:
//c  BEGIN EXPERIMENT WHEN CELL != STARTCELL
//c  ON SERVOLOOP() {
//c   TRACK DTF  [DISTANCE TO FINISH]
//c   RECORD DATA(T,X1,Y1,Z1,X2,Y2,Z2,CONTACT)
////c What is CONTACT?
//c   IF (DTF > MINDTF) CALL FEEDBACKWEAK
//c   IF (DTF <= MINDTF && METHOD = TEMP) MAKE WARM
//c   IF DTF = 1 EXIT LOOP
//c   IF (METHOD = FORCE) CALL DRIFTBACK
////c Should add another if statement for position guidance? (  IF (METHOD = POSITION) CALL DRIFTBACK  or something like: IF (DTF < MINDTF && METHOD = POSITION) CAL DRIFTBACK  )
//c  }
//c CLOSE DATA FILE
//c GOTO LOOPBACK

//c FEEDBACKWEAK = FUNCTION(METHOD) {
//c  IF (METHOD = NONE) RETURN
//c  TIMER TO APPLY FORCE EVERY X MILLISECONDS {
//c   IF (METHOD = FORCE) LEFTFORCE = FORCE(X1,Y1,Z1), an array which has the direction to the next cell
//c  }
//c  IF (METHOD = TEMP) MAKE COLD 
//c } 

//c DRIFTBACK = FUNCTION() {
//c  WEAKSIDEFORCE = K*(X2-X1, Y2-Y1, Z2-Z1)
//c }
//c NEW CODE ENDS *******************************************************************


void MotionCallback(unsigned int ShapeID)
{
}


/* BLOCK OLD

//void MotionCallback(unsigned int ShapeID);
void GraphicsCallback(void);

int i = 0;
bool hintsEnabled = false;
bool twoOmnis = true;

std::ofstream myFile;

int WINAPI WinMain(	HINSTANCE	hInstance,				// Instance
			HINSTANCE	hPrevInstance,				// Previous Instance
			LPSTR		lpCmdLine,				// Command Line Parameters
			int		nCmdShow)				// Window Show State
{
	
	QHWin32* DisplayObject1 = new QHWin32;//create a display window
    DisplayObject1->hapticWindow(true);//The haptics are with respect to this window
    DisplayObject1->setWindowTitle("Front View\0");//make the title of this window "Front View"

    QHWin32* DisplayObject2 = new QHWin32;//create a display window
    DisplayObject2->hapticWindow(false);//Disable haptics in this window
    DisplayObject2->setWindowTitle("Right View\0");//Set the title of this window as "Left View"

    QHWin32* DisplayObject3 = new QHWin32;//create a display window
    DisplayObject3->hapticWindow(false);//Disable Haptics in this window
    DisplayObject3->setWindowTitle("Top View\0");//Set the title of this window as "Top View"

	//DeviceSpace* OmniSpace = new DeviceSpace();//Find the default Phantom device 
    DeviceSpace* OmniSpace = new DeviceSpace("right");//Find the right Phantom device 
	DisplayObject1->tell(OmniSpace);//Tell QuickHaptics about it
	DeviceSpace* OmniSpace2;
	if(twoOmnis){
		OmniSpace2 = new DeviceSpace("left");//Find the left Phantom device 
		DisplayObject1->tell(OmniSpace2);
	}
	
	hduVector3Dd dirUp;
	dirUp[0]=0.0;
	dirUp[1]=1.0;
	dirUp[2]=0.0;
	OmniSpace->setConstantForce(dirUp,0.11); // Make weightless the Omni until hint force enabled, TODO
	if(twoOmnis){
		OmniSpace2->setConstantForce(dirUp,0.11); // Make weightless the Omni until hint force enabled, TODO
	}
	TriMesh* Maze = new TriMesh("models/Test2.3DS",1,0,-40,60);//Load a Maze model (Scale 1 LOC 0 -40 0)
	Maze->setUnDraggable();
	Maze->setName("Maze");//give it a name
	Maze->setShapeColor(0,0.404,0.278);
//	Maze->setPopthrough();
	//    Maze->setTexture("models/cow.jpg");
	
	Maze->dynamic(false);//make the Maze deformable
	Maze->setGravity(false);//Turn off gravity
//	Maze->setFriction(0.0,0.0);//give friction to the Maze surface
//	Maze->setSpringStiffness(0.5);//Parameters to play around with - Spring Stiffness
//	Maze->setSpringDamping(0.5);//Parameters to play around with - Damping
//	Maze->setMass(5);////Parameters to play around with - Mass of each particle

    DisplayObject1->tell(Maze);//Tell Quickhaptics about the Maze

//	Box* SimpleBox = new Box(138, 58, 98, 69, 1, -49); // DIM 140 60 100, LOC 70 30 -50
	Box* SimpleBox = new Box(140, 60, 100, 70, -10, 10); // DIM 140 60 100, LOC 70 -10 -50
	SimpleBox->setName("WOO");
	SimpleBox->setUnDraggable();
//    SimpleBox->setTexture("models/cow.jpg");
	SimpleBox->setTouchableFace("back");
	SimpleBox->setGraphicVisibility(false);
//	SimpleBox->setPopthrough();
	DisplayObject1->tell(SimpleBox);

    Text* descriptionText1 = new Text(18.0, "3D Maze", 0.2,0.9);
    descriptionText1->setName("TitleBox");
    descriptionText1->setShapeColor(0.0,0.0,0.0);
    DisplayObject1->tell(descriptionText1);
    Text* descriptionText2 = new Text(20.0, "0 0 0 - 0", 0.4,0.85);
    descriptionText2->setName("TextBox");
    descriptionText2->setShapeColor(1.0,0.8,0.9);
    DisplayObject1->tell(descriptionText2);
    Text* descriptionText3 = new Text(14.0, "000.0 ms", 0.4, 0.80);
	descriptionText3->setName("TimerBox");
    descriptionText3->setShapeColor(1.0,0.8,0.9);
    DisplayObject1->tell(descriptionText3);
	Text* descriptionText4 = new Text(14.0, "Under Construction", 0.4,0.75);
    descriptionText4->setName("CommentBox");
    descriptionText4->setShapeColor(1.0,0.2,0.0);
    DisplayObject1->tell(descriptionText4);
    
/*
    Cursor* OmniCursor = new Cursor;//Declare a new cursor
    OmniCursor->setName("OmniCursor");
    DisplayObject1->tell(OmniCursor);//tell QuickHaptics about the cursor
*/
/* BLOCK OLD
	Cursor* OmniCursor = new Cursor();//Declare a new cursor
    Cursor* OmniCursor2;
	OmniCursor->setName("OmniCursor");
	DisplayObject1->tell(OmniCursor);//tell QuickHaptics about the cursor
	if(twoOmnis){
	    OmniCursor2 = new Cursor();//Declare a new cursor
	    OmniCursor2->setName("OmniCursor2");
	    DisplayObject1->tell(OmniCursor2);//tell QuickHaptics about the cursor
	}

	DisplayObject1->preDrawCallback(GraphicsCallback);//set the graphics callback
  //  OmniSpace->motionCallback(MotionCallback, Maze);//set the movement callback (contact walls, TODO)
	// TODO
	//OmniSpace->startServoLoopCallback(startEffectCB, computeForceCB, stopEffectCB, &dataObject);

    float FOV1, FOV2, FOV3;
    float NearPlane1, NearPlane2, NearPlane3;
    float FarPlane1, FarPlane2, FarPlane3;
    hduVector3Dd Eye1, Eye2, Eye3;
    hduVector3Dd LookAt1, LookAt2, LookAt3;
    hduVector3Dd UpVector1, UpVector2, UpVector3;
  
    
    DisplayObject1->setDefaultCamera();
    DisplayObject1->getCamera(&FOV1,&NearPlane1,&FarPlane1,&Eye1,&LookAt1,&UpVector1);

    
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
  
	myFile.open("test.txt");
//	myFile << "File Opened" << std::endl ;
	qhStart();//Set everything in motion
//	myFile << "File Closed" << std::endl ;
	myFile.close();
}

void GraphicsCallback(void)
{
	// There's this timer, which is a bit of a hack job and is fairly inaccurate.
	// The servo callback needs to be active, which calls at a nearly precise 1kHz freq.
	// The beginnings of this are noted with "TODO" in the code above.
	// See p. 53 (2-27 Example 7) in the Programmer's Guide for a proper implementation.
	i++;
	Cursor* OmniCursorPointer = Cursor::searchCursor("OmniCursor");//Search for the cursor and return a pointer to it.
	Cursor* OmniCursorPointer2;
	if(twoOmnis){
		OmniCursorPointer2 = Cursor::searchCursor("OmniCursor2");//Search for the cursor and return a pointer to it.
	}
	//DeviceSpace* SpacePointer = DeviceSpace::searchSpace("Default PHANToM");//Search for the haptic device and return a pointer to it.
	DeviceSpace* SpacePointer = DeviceSpace::searchSpace("right");//Search for the haptic device and return a pointer to it.
	if(twoOmnis){
		DeviceSpace* SpacePointer2 = DeviceSpace::searchSpace("left");//Search for the haptic device and return a pointer to it.
	}
	Text* TextPointer = Text::searchText("TextBox");
	Text* TimerPointer = Text::searchText("TimerBox");

	hduVector3Dd CPosition;
	hduVector3Dd C2Position;
	CPosition = OmniCursorPointer->getPosition();//Get the current position of the haptic device
	if(twoOmnis){
		C2Position = OmniCursorPointer2->getPosition();//Get the current position of the haptic device
	}

	hduVector3Dd Force = SpacePointer->getForce();//Get the current force being exerted by the haptic device.
	hduVector3Dd Tracker;

	int PosX = CPosition[0]/20;
	int PosY = CPosition[1]/20+2;
	int PosZ = CPosition[2]/20+2;
	
	double FinePosX1 = CPosition[0]; // TODO WHY IS THIS RANGE DIFFERENT?!
	double FinePosY1 = CPosition[1];
	double FinePosZ1 = CPosition[2];

	double FinePosX2;
	double FinePosY2;
	double FinePosZ2;
	if(twoOmnis){
		FinePosX2 = C2Position[0];
		FinePosY2 = C2Position[1];
		FinePosZ2 = C2Position[2];
	}

	if(twoOmnis){
		myFile << " " << std::setw(10) << FinePosX2 << " " << std::setw(10) << FinePosY2 << " " << std::setw(10) << FinePosZ2 ;
	}
	myFile << std::setw(10) << i << " " << std::setw(10) << FinePosX1 << " " << std::setw(10) << FinePosY1 << " " << std::setw(10) << FinePosZ1;
	myFile << std::endl;

	const int dist[7][3][5] = { { { 14, 13, 12, 11, 10}, { -1, -1, -1, -1,  9}, { 36, 35, 34, -1,  8} },
							    { { -1, -1, 13, -1, -1}, { -1, -1, -1, -1, -1}, { -1, -1, 33, -1,  7} },
							    { { 32, -1, 14, 15, 16}, { 33, -1, -1, -1, -1}, { 34, -1, 32, -1,  6} },
							    { { 31, -1, -1, -1, 17}, { -1, -1, -1, -1, -1}, { -1, -1, 31, -1,  5} },
							    { { 30, -1, 20, 19, 18}, { 29, -1, -1, -1, -1}, { 28, 29, 30, -1,  4} },
							    { { -1, -1, 21, -1, -1}, { -1, -1, -1, -1, -1}, { 27, -1, -1, -1,  3} },
							    { { 24, 23, 22, -1,  0}, { 25, -1, -1, -1,  1}, { 26, 27, 28, -1,  2} } };

	const int hint[7][3][5] = { { {  2,  2,  2,  2,  8}, { -1, -1, -1, -1,  8}, {  2,  2, 32, -1, 32} },
							    { { -1, -1, 16, -1, -1}, { -1, -1, -1, -1, -1}, { -1, -1, 32, -1, 32} },
							    { { 32, -1, 16,  1,  1}, {  4, -1, -1, -1, -1}, {  4, -1, 32, -1, 32} },
							    { { 32, -1, -1, -1, 16}, { -1, -1, -1, -1, -1}, { -1, -1, 32, -1, 32} },
							    { {  8, -1,  2,  2, 16}, {  8, -1, -1, -1, -1}, { 32,  1,  1, -1, 32} },
							    { { -1, -1, 16, -1, -1}, { -1, -1, -1, -1, -1}, { 32, -1, -1, -1, 32} },
							    { {  2,  2, 16, -1,  0}, {  4, -1, -1, -1,  0}, {  4,  1,  1, -1,  4} } };
/* 128 = START
    64 = FINISH
    32 = X+ (RIGHT)
    16 = X-
     8 = Y+ (FRONT)
     4 = Y-
     2 = Z+ (DOWN)
     1 = Z-  */
	
/* BLOCK OLD
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
	stri << (i*5);
	std::string temp_str = strs.str();
	std::string timer_str = stri.str();
	char* pchar = (char*) temp_str.c_str();
	char* tchar = (char*) timer_str.c_str();
	TextPointer->update(pchar);	
	TimerPointer->update(tchar);
	if(hintsEnabled){
	int hintForce;
	if ((PosX > -1) & (PosX < 7) & (PosY > -1) & (PosY < 3) & (PosZ > -1) & (PosZ < 5))
	{
		hintForce = hint[PosX][PosY][PosZ];
	}
	else
	{
		hintForce = -1;
	}
	
	switch(hintForce){
		case 1:
			Tracker[0] = 0;
			Tracker[1] = 0  + 0.5;
			Tracker[2] = -1;
			break;
		case 2:
			Tracker[0] = 0;
			Tracker[1] = 0  + 0.5;
			Tracker[2] = 1;
			break;
		case 4:
			Tracker[0] = 0;
			Tracker[1] = -1  + 0.5;
			Tracker[2] = 0;
			break;
		case 8:
			Tracker[0] = 0;
			Tracker[1] = 1  + 0.5;
			Tracker[2] = 0;
			break;
		case 16:
			Tracker[0] = -1;
			Tracker[1] = 0  + 0.5;
			Tracker[2] = 0;
			break;
		case 32:
			Tracker[0] = 1;
			Tracker[1] = 0  + 0.5;
			Tracker[2] = 0;
			break;
		default:
			Tracker[0] = 0;
			Tracker[1] = 0  + 0.5;
			Tracker[2] = 0;
	}
	SpacePointer->setConstantForce(Tracker, 0.3);
	}
	// select hint
	//   find a good binary-based selection online. (Char?) Can start w/ n-128 and n-64 to start, and later do diagonals
	// end select
	/* ROUGH HINT SNIPPET BELOW (TODO)
	double magn=0.6;
	Force[0]=0.7;
	Force[1]=0.0;
	Force[2]=0.0;
	SpacePointer->setConstantForce(Force, magn);
	// (dir,magn);
    // */

	 /* TODO Attempt to follow position
	// Is there a different position vector for just the Omni in QuickHaptics???
	Tracker[0]=FinePosX2-FinePosX1;
	Tracker[1]=FinePosY2-FinePosY1;
	Tracker[2]=FinePosZ2-FinePosZ1;

	double magn = 0.05*pow((pow((double)Tracker[0],2)+pow((double)Tracker[1],2)+pow((double)Tracker[2],2)),0.5);
	strs << ":" << Tracker[0] << " " << Tracker[1] << " " << Tracker[2];
	temp_str = strs.str();
	pchar = (char*) temp_str.c_str();
	TextPointer->update(pchar);	
	
	SpacePointer->setConstantForce(Tracker, 0.6);
	// */

/* BLOCK OLD
	}

/*
void MotionCallback(unsigned int ShapeID)
{
    //Cursor* OmniCursorPointer = Cursor::searchCursor("OmniCursor");//Search for the cursor and return a pointer to it.
    //Cursor* OmniCursorPointer2 = Cursor::searchCursor("OmniCursor2");//Search for the cursor and return a pointer to it.
	//DeviceSpace* SpacePointer = DeviceSpace::searchSpace("Default PHANToM");//Search for the haptic device and return a pointer to it.
	//DeviceSpace* SpacePointer = DeviceSpace::searchSpace("right");//Search for the haptic device and return a pointer to it.
	//DeviceSpace* SpacePointer = DeviceSpace::searchSpace("left");//Search for the haptic device and return a pointer to it.
	
	//hduVector3Dd CPosition;
	//CPosition = OmniCursorPointer->getPosition();//Get the current position of the haptic device
	
	//hduVector3Dd Force = SpacePointer->getForce();//Get the current force being exerted by the haptic device.
}
*/