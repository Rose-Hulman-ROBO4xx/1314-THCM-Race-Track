#include <windows.h>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include "Leap.h"
#include "rs232.h"
using namespace Leap;


#define ROBOT_ARM_MOVE_PRECURSOR 'M'
#define HORIZONTAL_LEFT_COMMAND '2'
#define HORIZONTAL_NEUTRAL_COMMAND '0'
#define HORIZONTAL_RIGHT_COMMAND '1'
#define VERTICAL_UP_COMMAND '1'
#define VERTICAL_NEUTRAL_COMMAND '0'
#define VERTICAL_DOWN_COMMAND '2'
#define ROBOT_ARM_READY_POSITION_PRECURSOR 'R'
#define ROBOT_ARM_READY_POSITION '1'
#define ROBOT_ARM_HOME_POSITION_PRECURSOR 'R'
#define ROBOT_ARM_HOME_POSITION '0'
#define ROBOT_LASER_PRECURSOR 'L'
#define ROBOT_LASER_OFF '0'
#define ROBOT_LASER_ON '1'
#define NEWLINE '\n'
#define SPACE ' '

#define CAR_DATA_REQUEST_PRECURSOR 'Q'
#define CAR_GAS_SENSOR_DESIGNATOR 'G'
#define CAR_TIRE_SENSOR_DESIGNATOR 'T'
#define CAR_MOTOR_SPEED_PRECURSOR 'M'
#define CAR_MOTOR_SPEED_FAST 100
#define CAR_MOTOR_SPEED_MEDIUM 70
#define CAR_MOTOR_SPEED_SLOW 40
#define CAR_DRIVE 'D'
#define CAR_STOP 'S'
#define GAS_SENSOR_THRESHOLD 50
#define TIRE_SENSOR_THRESHOLD 25

#define CAR_PORT_NUMBER 5
#define ROBOT_PORT_NUMBER 6
#define CAR_BAUDRATE CBR_9600
#define ROBOT_BAUDRATE CBR_9600
#define CAR_BYTESIZE 8
#define CAR_STOPBITS ONESTOPBIT
#define CAR_PARITY NOPARITY

#define CLICK_COLOR sf::Color::Black
#define REST_COLOR sf::Color::White
#define CONTROL_CLICK_COLOR sf::Color::Green
#define CONTROL_REST_COLOR sf::Color::Yellow
#define RESET_CONFIRMATION_COLOR sf::Color::Red
#define RESET_CONFIRMATION_BUTTON_TEXT_COLOR sf::Color::White
#define RESET_BUTTON_CANCEL_CLICKED_COLOR sf::Color::Green
#define LOADING_COLOR sf::Color::Cyan
#define LOADING_TEXT_COLOR sf::Color::Black

#define ICON_SIZE sf::Vector2i(64,64)
#define FONT_SIZE 24
#define LOADING_TEXT_SIZE 24
 
//as milliseconds
#define HOLD_TIME_TO_CLICK 1000
#define TIME_TO_RESET 5000
#define TIME_TO_DELAY_MENUS 750
#define TIME_TO_DELAY_MENUS_FUDGE_FACTOR 5
#define TIME_TO_DELAY_MENUS_MOUSE_REDUCTION 7*TIME_TO_DELAY_MENUS/8
#define TIME_TO_CHANGE_LOADING_MESSAGE TIME_TO_STOP_LOADING/8
#define TIME_TO_STOP_LOADING 3000
#define TIME_TO_REDUCE_GAS 2000
#define TIME_TO_REDUCE_TIRES 1000
#define TIME_TO_CHECK_SENSORS 1000
#define TIME_TO_DELAY_REFRESH 100
#define TIME_TO_UPDATE_STATES 100
#define INITIALIZE_TIME 10000

#define SOUND_NONE 0
#define SOUND_CLICK 2^0
#define SOUND_ROBOT_GAS_PREP 2^1
#define SOUND_ROBOT_TIRE_PREP 2^2
#define SOUND_ROBOT_GAS_READY 2^3
#define SOUND_ROBOT_TIRE_READY 2^4
#define SOUND_ROBOT_GAS_DONE 2^5
#define SOUND_ROBOT_TIRE_DONE 2^6
#define SOUND_GAS_EMPTY 2^7
#define SOUND_TIRE_EMPTY 2^8
#define SOUND_GAS_NEAR_EMPTY 2^9
#define SOUND_TIRE_NEAR_EMPTY 2^10
#define SOUND_RESET 2^11
#define SOUND_LEAP_CONNECTION_ERROR 2^12
#define SOUND_ROBOT_CONNECTION_ERROR 2^13
#define SOUND_CAR_CONNECTION_ERROR 2^14

#define STATE_UPDATE_GAS_VALUE 0
#define STATE_UPDATE_TIRE_VALUE 1
#define STATE_UPDATE_MOTOR_SPEED 2
#define STATE_UPDATE_GAS_LEDS 3
#define STATE_UPDATE_TIRE_LEDS 4

#define BUTTON_AREA_WIDTH 2*width/5
#define BUTTON_AREA_HEIGHT height/2
#define DISPLAY_AREA_WIDTH width/2
#define DISPLAY_AREA_HEIGHT height/2
#define LEAP_BUTTON_WIDTH width/15
#define LEAP_BUTTON_HEIGHT height/15
#define MINUS_FORTY_FIVE_DEGREES -45
#define FORTY_FIVE_DEGREES 45
#define RIGHT_MARGIN 25
#define LEFT_MARGIN 25
#define BOTTOM_MARGIN 25
#define BUTTON_MARGIN 2
#define LEAP_CONTROL_X 3*palmPos.x + trueZero.x + width/2
#define LEAP_CONTROL_Y 2*height - (trueZero.y + 2*palmPos.y)
#define RESET_CONFIRMATION_MESSAGE_X_AXIS_CENTERING_FACTOR 0

#define GAS_WAIT_MESSAGE_1 "Preparing to fill the gas."
#define GAS_WAIT_MESSAGE_2 "Preparing to fill the gas.."
#define GAS_WAIT_MESSAGE_3 "Preparing to fill the gas..."
#define TIRE_WAIT_MESSAGE_1 "Preparing to change the tires."
#define TIRE_WAIT_MESSAGE_2 "Preparing to change the tires.."
#define TIRE_WAIT_MESSAGE_3 "Preparing to change the tires..."

#define RESET_CONFIRMATION_MESSAGE "Reset?"
#define RESET_COUNTDOWN_MESSAGE_5 "5..."
#define RESET_COUNTDOWN_MESSAGE_4 "4..."
#define RESET_COUNTDOWN_MESSAGE_3 "3..."
#define RESET_COUNTDOWN_MESSAGE_2 "2..."
#define RESET_COUNTDOWN_MESSAGE_1 "1..."
#define RESET_COUNTDOWN_MESSAGE_0 "0..."
#define RESET_CANCELLED_MESSAGE "Reset cancelled."
#define RESET_ACCEPT_BUTTON_MESSAGE "Yes"
#define RESET_CANCEL_BUTTON_MESSAGE "Cancel"

#define HELP_MESSAGE_GAS_GAUGE "Gas gauge"
#define HELP_MESSAGE_TIRE_GAUGE "Tire gauge"
#define HELP_MESSAGE_PIT_CREW "Pit crew"
#define HELP_MESSAGE_RESET_BUTTON "Reset button"
#define HELP_MESSAGE_GAS_BUTTON "Fill your gas"
#define HELP_MESSAGE_TIRE_BUTTON "Change tires"
#define HELP_MESSAGE_CREDITS_1 "Special thanks to:"
#define HELP_MESSAGE_CREDITS_2 "icon8"

#define LOADING_MESSAGE_3 "Loading..."
#define LOADING_MESSAGE_2 "Loading.."
#define LOADING_MESSAGE_1 "Loading."

#define LEAP_GREEN_MESSAGE "Ready!"
#define LEAP_RED_MESSAGE "Wave hand over controller to begin!"
#define LEAP_BLACK_MESSAGE "Controller is disconnected."
#define HELP_MESSAGE "Turn help off"


//Black-discon, Red-no focus, Green-focused
sf::Color leapColor = sf::Color::Black;


sf::Vector2i trueZero;
sf::Vector2i mousePos;

sf::RectangleShape selectedIcon;

Vector palmPos;

char horzCommand = HORIZONTAL_NEUTRAL_COMMAND;
char vertCommand = VERTICAL_NEUTRAL_COMMAND;

unsigned char tireValue = 10;
unsigned char gasValue = 10;
unsigned char motorUpdate = 0;
unsigned char motorSpeed = 0;
unsigned char carCommState = STATE_UPDATE_GAS_VALUE;
unsigned char oldGasValue = 0;
unsigned char oldTireValue = 0;

sf::Clock clickClock;
sf::Clock resetClock;
sf::Clock changeMenuClock;
sf::Clock robotLoadingClock;
sf::Clock robotStopLoadingClock; //Test purposes only
sf::Clock tireGaugeClock;
sf::Clock gasGaugeClock;
sf::Clock sensorClock;
sf::Clock refreshClock;
sf::Clock initialClock;
sf::Clock stateClock;

sf::SoundBuffer soundBuffer;
sf::Sound sound;

bool mousePressed = 0;
bool leapControl = 0;
bool robotArmActive = 0;
bool resetActive = 0;
bool helpActive = 0;
bool loadingActive = 0;
bool tireReset = 0;
bool gasReset = 0;
bool tireUpdate = 0;
bool gasUpdate = 0;
bool tireGame = 0;
bool gasGame = 0;
bool refreshRobotPosition = 0;
bool initialLoad = 1;
bool soundPlaying = 0;
bool driveCar = 0;
bool carIsDriving = 0;


int queuedSound = SOUND_NONE;


// Leap Motion Class
// Customized for use in the THCM project
//
class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
};

void SampleListener::onInit(const Controller& controller) {
	leapColor = sf::Color::Black;
}

void SampleListener::onConnect(const Controller& controller) {
	leapColor = sf::Color::Green;
}

void SampleListener::onDisconnect(const Controller& controller) {
    //Note: not dispatched when running in a debugger.
	leapColor = sf::Color::Black;
	queuedSound &= SOUND_LEAP_CONNECTION_ERROR;
} 

void SampleListener::onExit(const Controller& controller) {
	leapColor = sf::Color::Black;
}

void SampleListener::onFrame(const Controller& controller) {
	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();

	if (!frame.hands().isEmpty()) {
		// Get the first hand only
		const Hand hand = frame.hands()[0];
		palmPos = hand.palmPosition();
	}

}

void SampleListener::onFocusGained(const Controller& controller) {
	leapColor = sf::Color::Green;
}

void SampleListener::onFocusLost(const Controller& controller) {
	leapColor = sf::Color::Red;
}



//Returns the coordinates to center a rectangle inside of another
//   ***Assumes innerRect is smaller than outerRect
sf::Vector2i centerRectOnRect(sf::RectangleShape innerRect, sf::RectangleShape outerRect){

	//Get the position of the outer rectangle
	sf::Vector2f outerPosition = outerRect.getPosition();
	int posX = outerPosition.x;
	int posY = outerPosition.y;

	//Get the size of the outer rectangle
	sf::Vector2f outerSize = outerRect.getSize();
	int outerX = outerSize.x;
	int outerY = outerSize.y;

	//Get the size of the inner rectangle
	sf::Vector2f innerSize = innerRect.getSize();
	int innerX = innerSize.x;
	int innerY = innerSize.y;

	//Calculate the necessary shift to center
	int finalPosX = posX + (outerX - innerX)/2;
	int finalPosY = posY + (outerY - innerY)/2;

	//Move the inner rectangle to desired location
	return sf::Vector2i(finalPosX, finalPosY);

}

//Returns the coordinates to center a sprite inside of a rectangle
//   ***Assumes innerRect is smaller than outerRect
sf::Vector2i centerSpriteOnRect(sf::Vector2i spriteSize, sf::RectangleShape outerRect){

	//Get the position of the outer rectangle
	sf::Vector2f outerPosition = outerRect.getPosition();
	int posX = outerPosition.x;
	int posY = outerPosition.y;

	//Get the size of the outer rectangle
	sf::Vector2f outerSize = outerRect.getSize();
	int outerX = outerSize.x;
	int outerY = outerSize.y;

	//Get the size of the sprite
	int spriteX = spriteSize.x;
	int spriteY = spriteSize.y;

	//Calculate the necessary shift to center
	int finalPosX = posX + (outerX - spriteX)/2;
	int finalPosY = posY + (outerY - spriteY)/2;

	//Move the inner rectangle to desired location
	return sf::Vector2i(finalPosX, finalPosY);

}

//Returns coordinates for the new rectangle to be placed on 
//the desired side of the static rectangle.
// 'a' - above, 'l' - left, 'r' - right, 'b' or anything else - below.
sf::Vector2f placeRectNextToRect(sf::RectangleShape staticRect, sf::RectangleShape borderRect, char direction){

	//Get the static rectangle's size
	sf::Vector2f staticSize = staticRect.getSize();
	int staticSizeX = ceil(staticSize.x);
	int staticSizeY = ceil(staticSize.y);

	//Get the static rectangle's position
	sf::Vector2f staticPos = staticRect.getPosition();
	int staticPosX = ceil(staticPos.x);
	int staticPosY = ceil(staticPos.y);

	//Get the border rectangle's size
	sf::Vector2f borderSize =  borderRect.getSize();
	int borderSizeX = ceil(borderSize.x);
	int borderSizeY = ceil(borderSize.y);

	sf::Vector2f newPos;

	if(direction == 'a')
	//Place above
	{
		newPos = sf::Vector2f(staticPosX, staticPosY - borderSizeY - BUTTON_MARGIN);
	}
	else if(direction == 'l')
	//Place to the left
	{
		newPos = sf::Vector2f(staticPosX - borderSizeX - BUTTON_MARGIN, staticPosY);
	}
	else if(direction == 'r')
	//Place to the right
	{
		newPos = sf::Vector2f(staticPosX + staticSizeX + BUTTON_MARGIN, staticPosY);
	}
	else 
	//Place below
	{
		//newPos = sf::Vector2f(staticPosX>200?staticPosX-1:staticPosX, staticPosY + staticSizeY + 2);
		newPos = sf::Vector2f(staticPosX, staticPosY + staticSizeY + BUTTON_MARGIN);
	}

	return newPos;

}

//Returns TRUE iff mousePos is within the rectangle given
bool isMouseInRect(sf::RectangleShape rectangle){

	//Retrieve the size and position of the rectangle
	sf::Vector2f rectPos = rectangle.getPosition();
	sf::Vector2f rectSize = rectangle.getSize();

	//return TRUE iff the mouse is within the boundary
	return 
		mousePos.x >= rectPos.x && 
		mousePos.y >= rectPos.y && 
		mousePos.x <= (rectPos.x + rectSize.x) && 
		mousePos.y <= (rectPos.y + rectSize.y);

}

//Resets the gauges
void resetGame(){
	resetActive = 0;
	gasReset = 1;
	tireReset = 1;
	gasGaugeClock.restart();
	tireGaugeClock.restart();
}

void updateLeapMessage(sf::Text leapMessage){
	
	leapMessage.setColor(leapColor);

	if(leapColor == sf::Color::Green)
		leapMessage.setString(LEAP_GREEN_MESSAGE);
	else if(leapColor == sf::Color::Red)
		leapMessage.setString(LEAP_RED_MESSAGE);
	else 
		leapMessage.setString(LEAP_BLACK_MESSAGE);
}

//Request and read the CDS sensor data for the gas value
//Returns the value provided by the PIC in the car
// This value is currently a single char (8 bits)
unsigned char readGasData(){
	
	unsigned char writeToCar = CAR_DATA_REQUEST_PRECURSOR;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	writeToCar = CAR_GAS_SENSOR_DESIGNATOR;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	writeToCar = NEWLINE;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		
	unsigned char readFromCar = 0;
	RS232_PollComport(CAR_PORT_NUMBER, &readFromCar, 1);

	return readFromCar;
}

//Adjust the read sensor data for the gas value
//Returns a normalized value to use for the gas gauge
// This value is currently boolean
boolean adjustGasData(unsigned char sensorValue){

	//Return 1 iff the gas sensor reads higher than the threshold
	return sensorValue > GAS_SENSOR_THRESHOLD;

	//This setup handles a multiple-value system, 
	//  where the sensor interprets the current value 
	//  and returns the necessary value based on current readings
	// Deemed potentially helpful, but not for the current design of the system.
	/***************
	int thresholdDividend = (GAS_SENSOR_MAX_VALUE - GAS_SENSOR_MIN_VALUE)/GAS_SENSOR_NUM_THRESHOLDS;
	int currentThreshold = thresholdDividend, 
	unsigned char currentCount = 0;

	
	while(currentThreshold < GAS_SENSOR_MAX_VALUE){
		if(sensorValue > currentThreshold){ 
			currentThreshold += thresholdDividend;
			currentCount++;
		}
		else break;
	}

	return currentCount;
	*****************/
}

//Handle all the necessary steps to update the gas gauge
// Count down the gas value
// If in the loading screen, continue counting down the value
// If gasReset is active (1), reset to 'full'.
// If the mini-game is in progress (robotArmActive), 
//    read the value from the sensor and update the gas gauge 
//    incrementally.  This simulates the gas tank filling.
void updateGasValue(){
	if(gasReset && !loadingActive && !robotArmActive){
		//Reset the gas value
		gasReset = 0;
		gasValue = 10;
		gasUpdate = 1;
		
		unsigned char writeToRobot = ROBOT_LASER_PRECURSOR;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = SPACE;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = ROBOT_LASER_ON;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = NEWLINE;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		
		driveCar = 1;

	} else if(!robotArmActive){
		//Decrement the gas value
		if(gasGaugeClock.getElapsedTime().asMilliseconds() > TIME_TO_REDUCE_GAS){
			if(gasValue != 0 && !initialLoad){
				gasValue--;
				gasUpdate = 1;
			}
			gasGaugeClock.restart();
			driveCar = 1;
		}
	} else if(robotArmActive){
		// Increment the gas value iff the sensor is 'active'
		
		driveCar = 0;

		unsigned char writeToRobot = ROBOT_LASER_PRECURSOR;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = SPACE;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = ROBOT_LASER_ON;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
		writeToRobot = NEWLINE;
		RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);

		if(sensorClock.getElapsedTime().asMilliseconds() > TIME_TO_CHECK_SENSORS && gasGame){
			if(adjustGasData(readGasData())){ 
				if(gasValue!=10){
					gasValue++;
					gasUpdate = 1;
				}
			}
			sensorClock.restart();
		}
	}
}

//Request and read the SRF sensor data for the tire value
//Returns the value provided by the PIC in the car
// This value is currently a single char (8 bits)
unsigned char readTireData(){
	
	unsigned char writeToCar = CAR_DATA_REQUEST_PRECURSOR;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	writeToCar = CAR_TIRE_SENSOR_DESIGNATOR;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	writeToCar = NEWLINE;
	RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	
	unsigned char readFromCar = 255;
	RS232_PollComport(CAR_PORT_NUMBER, &readFromCar, 1);

	return readFromCar;
}

//Adjust the read sensor data for the tire value
//Returns a normalized value to use for the tire gauge
// This value is currently boolean
boolean adjustTireData(unsigned char sensorValue){

	//Return TRUE iff the tire sensor reads lower than the threshold
	return sensorValue < TIRE_SENSOR_THRESHOLD;

}

//Handle all the necessary steps to update the tire gauge
// Count down the tire value
// If in the loading screen, continue counting down the value
// If tireReset is active (1), reset to 'full'.
// If the mini-game is in progress (robotArmActive), 
//    read the value from the sensor and update the tire gauge 
//    incrementally.  This simulates the tires being changed.
void updateTireValue(){

	if(tireReset && !loadingActive && !robotArmActive){
		//Reset the tire value
		tireReset = 0;
		tireValue = 10;
		tireUpdate = 1;

		driveCar = 1;

	} else if(!robotArmActive){
		//Decrement the tire value
		if(tireGaugeClock.getElapsedTime().asMilliseconds() > TIME_TO_REDUCE_TIRES){
			if(tireValue != 0 && !initialLoad){
				tireValue--;
				tireUpdate = 1;
			}
			tireGaugeClock.restart();
			driveCar = 1;
		}
	} else if(robotArmActive){
		// Increment the tire value iff the sensor is 'active'
		driveCar = 0;

		if(sensorClock.getElapsedTime().asMilliseconds() > TIME_TO_CHECK_SENSORS && tireGame){
			if(adjustTireData(readTireData())){ 
				if(tireValue != 10){
					tireValue++;
					tireUpdate = 1;
				}
			}
			sensorClock.restart();
		}
	}
}

//Update the gas LED colors on the car
void updateGasLEDs(){

	if(oldGasValue != gasValue){

		oldGasValue = gasValue;
	
		unsigned char writeToCar = CAR_GAS_SENSOR_DESIGNATOR;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = 10*(10-gasValue)+1; //Red Value
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = 10*(gasValue)+1; //Green Value
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = NEWLINE;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	}
}

//Update the tire LED colors on the car
void updateTireLEDs(){

	if(oldTireValue != tireValue){

		oldTireValue = tireValue;
	
		unsigned char writeToCar = CAR_TIRE_SENSOR_DESIGNATOR;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = 10*(10-tireValue)+1; //Red Value
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = 10*(tireValue)+1; //Green Value
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = NEWLINE;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
	}
}

//Update the motor speed on the car
void updateMotorSpeed(){

	if(tireValue < 2 || gasValue < 2){
		motorUpdate = CAR_MOTOR_SPEED_SLOW;

	} else if(tireValue < 7){
		motorUpdate = CAR_MOTOR_SPEED_MEDIUM;

	} else {
		motorUpdate = CAR_MOTOR_SPEED_FAST;
	}

	if(motorSpeed != motorUpdate){
			
		unsigned char writeToCar = CAR_MOTOR_SPEED_PRECURSOR;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = motorUpdate;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
		writeToCar = NEWLINE;
		RS232_SendByte(CAR_PORT_NUMBER, writeToCar);

		motorSpeed = motorUpdate;
	}
}

//Update the sound output
//	If a new state has been entered,
//  play the appropriate sound.
int updateSound(){

/*
#define SOUND_NONE 0
#define SOUND_CLICK 2^0
#define SOUND_ROBOT_GAS_PREP 2^1
#define SOUND_ROBOT_TIRE_PREP 2^2
#define SOUND_ROBOT_GAS_READY 2^3
#define SOUND_ROBOT_TIRE_READY 2^4
#define SOUND_ROBOT_GAS_DONE 2^5
#define SOUND_ROBOT_TIRE_DONE 2^6
#define SOUND_GAS_EMPTY 2^7
#define SOUND_TIRE_EMPTY 2^8
#define SOUND_GAS_NEAR_EMPTY 2^9
#define SOUND_TIRE_NEAR_EMPTY 2^10
#define SOUND_RESET 2^11
#define SOUND_LEAP_CONNECTION_ERROR 2^12
#define SOUND_ROBOT_CONNECTION_ERROR 2^13
#define SOUND_CAR_CONNECTION_ERROR 2^14
*/

	if(queuedSound & SOUND_NONE){
		//Do nothing
	}
	else if(queuedSound & SOUND_CLICK){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Click.wav"))
		{ //Error
			return SOUND_CLICK;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_CLICK;
	}
	else if(queuedSound & SOUND_ROBOT_GAS_PREP){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Gas_Prep.wav"))
		{ //Error
			return SOUND_ROBOT_GAS_PREP;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_GAS_PREP;
	}
	else if(queuedSound & SOUND_ROBOT_TIRE_PREP){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Tire_Prep.wav"))
		{ //Error
			return SOUND_ROBOT_TIRE_PREP;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_TIRE_PREP;
	}
	else if(queuedSound & SOUND_ROBOT_GAS_READY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Crew_Ready.wav"))
		{ //Error
			return SOUND_ROBOT_GAS_READY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_GAS_READY;
	}
	else if(queuedSound & SOUND_ROBOT_TIRE_READY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Crew_Ready.wav"))
		{ //Error
			return SOUND_ROBOT_TIRE_READY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_TIRE_READY;
	}
	else if(queuedSound & SOUND_ROBOT_GAS_DONE){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Crew_Ready.wav"))
		{ //Error
			return SOUND_ROBOT_GAS_DONE;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_GAS_DONE;
	}
	else if(queuedSound & SOUND_ROBOT_TIRE_DONE){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Crew_Ready.wav"))
		{ //Error
			return SOUND_ROBOT_TIRE_DONE;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_TIRE_DONE;
	}
	else if(queuedSound & SOUND_GAS_EMPTY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Gas_Empty.wav"))
		{ //Error
			return SOUND_GAS_EMPTY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_GAS_EMPTY;
	}
	else if(queuedSound & SOUND_TIRE_EMPTY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Tire_Empty.wav"))
		{ //Error
			return SOUND_TIRE_EMPTY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_TIRE_EMPTY;
	}
	else if(queuedSound & SOUND_GAS_NEAR_EMPTY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Gas_Near_Empty.wav"))
		{ //Error
			return SOUND_GAS_NEAR_EMPTY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_GAS_NEAR_EMPTY;
	}
	else if(queuedSound & SOUND_TIRE_NEAR_EMPTY){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Tire_Near_Empty.wav"))
		{ //Error
			return SOUND_TIRE_NEAR_EMPTY;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_TIRE_NEAR_EMPTY;
	}
	else if(queuedSound & SOUND_RESET){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Click.wav"))
		{ //Error
			return SOUND_RESET;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_RESET;
	}
	else if(queuedSound & SOUND_LEAP_CONNECTION_ERROR){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Leap_Error.wav"))
		{ //Error
			return SOUND_LEAP_CONNECTION_ERROR;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_LEAP_CONNECTION_ERROR;
	}
	else if(queuedSound & SOUND_ROBOT_CONNECTION_ERROR){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Robot_Serial_Error.wav"))
		{ //Error
			return SOUND_ROBOT_CONNECTION_ERROR;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_ROBOT_CONNECTION_ERROR;
	}
	else if(queuedSound & SOUND_CAR_CONNECTION_ERROR){
		if(!soundBuffer.loadFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\Car_Serial_Error.wav"))
		{ //Error
			return SOUND_CAR_CONNECTION_ERROR;
		}
		sound.setBuffer(soundBuffer);
		sound.play();
		queuedSound ^= SOUND_CAR_CONNECTION_ERROR;
	}
	else;

	return 0;
}



int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int nCmdShow)
{
	initialClock.restart();
	robotLoadingClock.restart();

	//Create the window and capture the size
	sf::RenderWindow window(sf::VideoMode(1280, 720), "Race Car Game",sf::Style::Default);
	sf::Vector2u size = window.getSize();
	unsigned int width = size.x;
	unsigned int height = size.y;
	trueZero = window.getPosition();

	// Declare a new music
	sf::Music music;
	// Open it from an audio file
	if (!music.openFromFile("C:\\Users\\mccullwc\\Documents\\Courses\\ROBO420\\Sounds and Music\\500-Chunks.wav"))
	{
		return 42;
	}
	// Change some parameters
	music.setVolume(50);
	music.setLoop(true);         // make it loop
	// Play it
	//music.play();


	//Create the elements desired
	sf::RectangleShape backgroundRectangle(sf::Vector2f(width, height));
	
	sf::RectangleShape gasRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/2, BUTTON_AREA_HEIGHT/2));
	sf::RectangleShape tireRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/2, BUTTON_AREA_HEIGHT/2));
	sf::RectangleShape helpRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/2, BUTTON_AREA_HEIGHT/2));
	sf::RectangleShape handRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/2, BUTTON_AREA_HEIGHT/2));
	
	sf::RectangleShape leapRectangle(sf::Vector2f(LEAP_BUTTON_WIDTH, LEAP_BUTTON_HEIGHT));
	
	sf::RectangleShape tireGaugeRectangle(sf::Vector2f(DISPLAY_AREA_WIDTH/2, DISPLAY_AREA_HEIGHT/2));
	sf::RectangleShape gasGaugeRectangle(sf::Vector2f(DISPLAY_AREA_WIDTH/2, DISPLAY_AREA_HEIGHT/2));
	sf::RectangleShape crewRectangle(sf::Vector2f(DISPLAY_AREA_WIDTH + BUTTON_MARGIN, DISPLAY_AREA_HEIGHT/2));
	
	sf::RectangleShape robotUpRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotUpLeftRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotUpRightRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotStopRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotLeftRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotRightRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotDownRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotDownLeftRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotDownRightRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	sf::RectangleShape robotExitRectangle(sf::Vector2f(BUTTON_AREA_WIDTH/3, BUTTON_AREA_HEIGHT/3));
	
	sf::RectangleShape resetConfirmationTextRectangle(sf::Vector2f(FONT_SIZE, FONT_SIZE));
	sf::RectangleShape resetConfirmationButtonRectangle(sf::Vector2f(8*FONT_SIZE, 2*FONT_SIZE));
	sf::RectangleShape resetConfirmationCountdownRectangle(sf::Vector2f(FONT_SIZE, FONT_SIZE));
	
	sf::RectangleShape robotLoadingRectangle(sf::Vector2f(BUTTON_AREA_WIDTH, BUTTON_AREA_HEIGHT));
	

	//Set the initial positions of the elements
	sf::Vector2i centerPoint = centerRectOnRect(gasRectangle, backgroundRectangle);
	gasGaugeRectangle.setPosition(LEFT_MARGIN, DISPLAY_AREA_HEIGHT/2);
	tireGaugeRectangle.setPosition(placeRectNextToRect(gasGaugeRectangle, tireGaugeRectangle,'r'));
	crewRectangle.setPosition(placeRectNextToRect(gasGaugeRectangle, crewRectangle,'b'));
	
	leapRectangle.setPosition(RIGHT_MARGIN, height - LEAP_BUTTON_HEIGHT - BOTTOM_MARGIN);

	tireRectangle.setPosition(width - RIGHT_MARGIN - BUTTON_AREA_WIDTH/2, BUTTON_AREA_HEIGHT/2);
	gasRectangle.setPosition(placeRectNextToRect(tireRectangle, gasRectangle,'l'));
	helpRectangle.setPosition(placeRectNextToRect(gasRectangle, helpRectangle,'b'));
	handRectangle.setPosition(placeRectNextToRect(helpRectangle, handRectangle,'r'));
	
	robotUpLeftRectangle.setPosition(gasRectangle.getPosition());
	robotUpRectangle.setPosition(placeRectNextToRect(robotUpLeftRectangle, robotUpRectangle,'r'));
	robotUpRightRectangle.setPosition(placeRectNextToRect(robotUpRectangle, robotUpRightRectangle,'r'));
	robotLeftRectangle.setPosition(placeRectNextToRect(robotUpLeftRectangle, robotLeftRectangle,'b'));
	robotStopRectangle.setPosition(placeRectNextToRect(robotLeftRectangle, robotStopRectangle,'r'));
	robotRightRectangle.setPosition(placeRectNextToRect(robotStopRectangle, robotRightRectangle,'r'));
	robotDownLeftRectangle.setPosition(placeRectNextToRect(robotLeftRectangle, robotDownLeftRectangle,'b'));
	robotDownRectangle.setPosition(placeRectNextToRect(robotDownLeftRectangle, robotDownRectangle,'r'));
	robotDownRightRectangle.setPosition(placeRectNextToRect(robotDownRectangle, robotDownRightRectangle,'r'));
	robotExitRectangle.setPosition(placeRectNextToRect(robotDownRectangle, robotExitRectangle,'b'));
	
	resetConfirmationTextRectangle.setPosition(centerRectOnRect(resetConfirmationTextRectangle, backgroundRectangle).x - RESET_CONFIRMATION_MESSAGE_X_AXIS_CENTERING_FACTOR, height - 3*resetConfirmationTextRectangle.getSize().y - BOTTOM_MARGIN);
	
	sf::Vector2i resetConfirmationButtonRectangleCenter = centerRectOnRect(resetConfirmationButtonRectangle , resetConfirmationTextRectangle);
	resetConfirmationButtonRectangle.setPosition(resetConfirmationButtonRectangleCenter.x + RESET_CONFIRMATION_MESSAGE_X_AXIS_CENTERING_FACTOR, resetConfirmationButtonRectangleCenter.y + resetConfirmationButtonRectangle.getSize().y);

	sf::Vector2f resetConfirmationCountdownRectangleCenter = placeRectNextToRect(resetConfirmationTextRectangle, resetConfirmationCountdownRectangle, 'r');
	resetConfirmationCountdownRectangle.setPosition(resetConfirmationCountdownRectangleCenter.x + 2*resetConfirmationTextRectangle.getSize().x, resetConfirmationCountdownRectangleCenter.y);

	sf::Vector2f robotLoadingRectangleCenter = gasRectangle.getPosition();
	robotLoadingRectangle.setPosition(robotLoadingRectangleCenter.x, robotLoadingRectangleCenter.y);

	//Set the initial fill color of the elements
	backgroundRectangle.setFillColor(sf::Color::Black);
    
	gasRectangle.setFillColor(sf::Color::Magenta);
	tireRectangle.setFillColor(sf::Color::Green);
	helpRectangle.setFillColor(sf::Color::White);
	handRectangle.setFillColor(sf::Color::Red);
	
	leapRectangle.setFillColor(sf::Color::White);
	tireGaugeRectangle.setFillColor(sf::Color::Green);
	gasGaugeRectangle.setFillColor(sf::Color::Magenta);
	crewRectangle.setFillColor(sf::Color::Cyan);

	robotUpLeftRectangle.setFillColor(CONTROL_REST_COLOR);
	robotUpRectangle.setFillColor(CONTROL_REST_COLOR);
	robotUpRightRectangle.setFillColor(CONTROL_REST_COLOR);
	robotLeftRectangle.setFillColor(CONTROL_REST_COLOR);
	robotStopRectangle.setFillColor(CONTROL_REST_COLOR);
	robotRightRectangle.setFillColor(CONTROL_REST_COLOR);
	robotDownLeftRectangle.setFillColor(CONTROL_REST_COLOR);
	robotDownRectangle.setFillColor(CONTROL_REST_COLOR);
	robotDownRightRectangle.setFillColor(CONTROL_REST_COLOR);
	robotExitRectangle.setFillColor(CONTROL_REST_COLOR);
	
	resetConfirmationTextRectangle.setFillColor(sf::Color::Transparent);
	resetConfirmationButtonRectangle.setFillColor(RESET_CONFIRMATION_COLOR);
	resetConfirmationCountdownRectangle.setFillColor(sf::Color::Transparent);
	
	robotLoadingRectangle.setFillColor(LOADING_COLOR);

	//Set the background texture and sprite
	sf::Texture backgroundTexture;
	if (!backgroundTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\Windows8Texture7-1280x800.png"))
	{}// error...

	sf::Sprite backgroundSprite;
	backgroundSprite.setTexture(backgroundTexture);
	
	//Set the gas icon texture and sprite
	sf::Texture gasTexture;
	if (!gasTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64.png"))
	{}// error...

	sf::Sprite gasSprite;
	gasSprite.setTexture(gasTexture);
	gasSprite.setColor(sf::Color::White);
	sf::Vector2i gasSpriteCenter = centerSpriteOnRect(ICON_SIZE, gasRectangle);
	gasSprite.setPosition(sf::Vector2f((float)gasSpriteCenter.x, (float)gasSpriteCenter.y));
			
	//Set the tire icon texture and sprite
	sf::Texture tireTexture;
	if (!tireTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64.png"))
	{}// error...

	sf::Sprite tireSprite;
	tireSprite.setTexture(tireTexture);
	tireSprite.setColor(sf::Color::White);
	sf::Vector2i tireSpriteCenter = centerSpriteOnRect(ICON_SIZE, tireRectangle);
	tireSprite.setPosition(sf::Vector2f((float)tireSpriteCenter.x, (float)tireSpriteCenter.y));
			
	//Set the hand icon texture and sprite
	sf::Texture handTexture;
	if (!handTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\hand_cursor-64.png"))
	{}// error...

	sf::Sprite handSprite;
	handSprite.setTexture(handTexture);
	handSprite.setColor(sf::Color::White);
	sf::Vector2i handSpriteCenter = centerSpriteOnRect(ICON_SIZE, handRectangle);
	handSprite.setPosition(sf::Vector2f((float)handSpriteCenter.x, (float)handSpriteCenter.y));
			
	//Set the help icon texture and sprite
	sf::Texture helpTexture;
	if (!helpTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\help-64.png"))
	{}// error...

	sf::Sprite helpSprite;
	helpSprite.setTexture(helpTexture);
	helpSprite.setColor(sf::Color::White);
	sf::Vector2i helpSpriteCenter = centerSpriteOnRect(ICON_SIZE, helpRectangle);
	helpSprite.setPosition(sf::Vector2f((float)helpSpriteCenter.x, (float)helpSpriteCenter.y));
			
	//Set the leap icon texture and sprite
	sf::Texture leapTexture;
	if (!leapTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\almost_full-64.png"))
	{}// error...

	sf::Sprite leapSprite;
	leapSprite.setTexture(leapTexture);
	leapSprite.setColor(leapColor);
	sf::Vector2i leapSpriteCenter = centerSpriteOnRect(ICON_SIZE, leapRectangle);
	leapSprite.setPosition(sf::Vector2f((float)leapSpriteCenter.x, (float)leapSpriteCenter.y));
			
	//Set the crew icon texture and sprite
	sf::Texture crewTexture;
	if (!crewTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\manager-64.png"))
	{}// error...

	sf::Sprite crewSprite;
	crewSprite.setTexture(crewTexture);
	crewSprite.setColor(sf::Color::White);
	sf::Vector2i crewSpriteCenter = centerSpriteOnRect(ICON_SIZE, crewRectangle);
	crewSprite.setPosition(sf::Vector2f((float)crewSpriteCenter.x, (float)crewSpriteCenter.y));
			
	//Set the gas gauge icon texture and sprite
	sf::Texture gasGaugeTexture;
	if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_10.png"))
	{}// error...

	sf::Sprite gasGaugeSprite;
	gasGaugeSprite.setTexture(gasGaugeTexture);
	gasGaugeSprite.setColor(sf::Color::White);
	sf::Vector2i gasGaugeSpriteCenter = centerSpriteOnRect(ICON_SIZE, gasGaugeRectangle);
	gasGaugeSprite.setPosition(sf::Vector2f((float)gasGaugeSpriteCenter.x, (float)gasGaugeSpriteCenter.y));
			
	//Set the tire gauge icon texture and sprite
	sf::Texture tireGaugeTexture;
	if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64.png"))
	{}// error...

	sf::Sprite tireGaugeSprite;
	tireGaugeSprite.setTexture(tireGaugeTexture);
	tireGaugeSprite.setColor(sf::Color::White);
	sf::Vector2i tireGaugeSpriteCenter = centerSpriteOnRect(ICON_SIZE, tireGaugeRectangle);
	tireGaugeSprite.setPosition(sf::Vector2f((float)tireGaugeSpriteCenter.x, (float)tireGaugeSpriteCenter.y));
	
	/*
	//Set the standard arrow icon texture and sprite
	sf::Texture robotArrowTexture;
	if (!robotArrowTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\up-64.png"))
	{}// error...

	sf::Sprite robotArrow;
	robotArrow.setTexture(robotArrowTexture);
	robotArrow.setColor(sf::Color::White);
	sf::Vector2i robotArrowCenter = centerSpriteOnRect(ICON_SIZE, robotUpLeftRectangle);
	robotArrow.setPosition(sf::Vector2f((float)robotArrowCenter.x, (float)robotArrowCenter.y));
	*/

	//Set the up left icon texture and sprite
	sf::Texture robotUpLeftTexture;
	if (!robotUpLeftTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\up_left-64.png"))
	{}// error...

	sf::Sprite robotUpLeftSprite;
	robotUpLeftSprite.setTexture(robotUpLeftTexture);
	robotUpLeftSprite.setColor(sf::Color::White);
	sf::Vector2i robotUpLeftSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotUpLeftRectangle);
	robotUpLeftSprite.setPosition(sf::Vector2f((float)robotUpLeftSpriteCenter.x, (float)robotUpLeftSpriteCenter.y));
	
	//Set the up icon texture and sprite
	sf::Texture robotUpTexture;
	if (!robotUpTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\up-64.png"))
	{}// error...

	sf::Sprite robotUpSprite;
	robotUpSprite.setTexture(robotUpTexture);
	robotUpSprite.setColor(sf::Color::White);
	sf::Vector2i robotUpSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotUpRectangle);
	robotUpSprite.setPosition(sf::Vector2f((float)robotUpSpriteCenter.x, (float)robotUpSpriteCenter.y));
	
	//Set the up right icon texture and sprite
	sf::Texture robotUpRightTexture;
	if (!robotUpRightTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\up_right-64.png"))
	{}// error...

	sf::Sprite robotUpRightSprite;
	robotUpRightSprite.setTexture(robotUpRightTexture);
	robotUpRightSprite.setColor(sf::Color::White);
	sf::Vector2i robotUpRightSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotUpRightRectangle);
	robotUpRightSprite.setPosition(sf::Vector2f((float)robotUpRightSpriteCenter.x, (float)robotUpRightSpriteCenter.y));
	
	//Set the left icon texture and sprite
	sf::Texture robotLeftTexture;
	if (!robotLeftTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\left-64.png"))
	{}// error...

	sf::Sprite robotLeftSprite;
	robotLeftSprite.setTexture(robotLeftTexture);
	robotLeftSprite.setColor(sf::Color::White);
	sf::Vector2i robotLeftSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotLeftRectangle);
	robotLeftSprite.setPosition(sf::Vector2f((float)robotLeftSpriteCenter.x, (float)robotLeftSpriteCenter.y));
	
	//Set the stop icon texture and sprite
	sf::Texture robotStopTexture;
	if (!robotStopTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\hand_cursor-64.png"))
	{}// error...

	sf::Sprite robotStopSprite;
	robotStopSprite.setTexture(robotStopTexture);
	robotStopSprite.setColor(sf::Color::White);
	sf::Vector2i robotStopSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotStopRectangle);
	robotStopSprite.setPosition(sf::Vector2f((float)robotStopSpriteCenter.x, (float)robotStopSpriteCenter.y));
	
	//Set the right icon texture and sprite
	sf::Texture robotRightTexture;
	if (!robotRightTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\right-64.png"))
	{}// error...

	sf::Sprite robotRightSprite;
	robotRightSprite.setTexture(robotRightTexture);
	robotRightSprite.setColor(sf::Color::White);
	sf::Vector2i robotRightSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotRightRectangle);
	robotRightSprite.setPosition(sf::Vector2f((float)robotRightSpriteCenter.x, (float)robotRightSpriteCenter.y));
	
	//Set the down left icon texture and sprite
	sf::Texture robotDownLeftTexture;
	if (!robotDownLeftTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\down_left-64.png"))
	{}// error...

	sf::Sprite robotDownLeftSprite;
	robotDownLeftSprite.setTexture(robotDownLeftTexture);
	robotDownLeftSprite.setColor(sf::Color::White);
	sf::Vector2i robotDownLeftSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotDownLeftRectangle);
	robotDownLeftSprite.setPosition(sf::Vector2f((float)robotDownLeftSpriteCenter.x, (float)robotDownLeftSpriteCenter.y));
	
	//Set the down icon texture and sprite
	sf::Texture robotDownTexture;
	if (!robotDownTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\down-64.png"))
	{}// error...

	sf::Sprite robotDownSprite;
	robotDownSprite.setTexture(robotDownTexture);
	robotDownSprite.setColor(sf::Color::White);
	sf::Vector2i robotDownSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotDownRectangle);
	robotDownSprite.setPosition(sf::Vector2f((float)robotDownSpriteCenter.x, (float)robotDownSpriteCenter.y));
	
	//Set the down right icon texture and sprite
	sf::Texture robotDownRightTexture;
	if (!robotDownRightTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\down_right-64.png"))
	{}// error...

	sf::Sprite robotDownRightSprite;
	robotDownRightSprite.setTexture(robotDownRightTexture);
	robotDownRightSprite.setColor(sf::Color::White);
	sf::Vector2i robotDownRightSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotDownRightRectangle);
	robotDownRightSprite.setPosition(sf::Vector2f((float)robotDownRightSpriteCenter.x, (float)robotDownRightSpriteCenter.y));
	
	//Set the exit icon texture and sprite
	sf::Texture robotExitTexture;
	if (!robotExitTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\backspace-64.png"))
	{}// error...

	sf::Sprite robotExitSprite;
	robotExitSprite.setTexture(robotExitTexture);
	robotExitSprite.setColor(sf::Color::White);
	sf::Vector2i robotExitSpriteCenter = centerSpriteOnRect(ICON_SIZE, robotExitRectangle);
	robotExitSprite.setPosition(sf::Vector2f((float)robotExitSpriteCenter.x, (float)robotExitSpriteCenter.y));
			
	
	
	//Set the texts
	sf::Font font;
	if(!font.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\CETIMES.ttf"))
	{//Error...
	}
	
	sf::Text resetText;
	resetText.setFont(font);
	resetText.setCharacterSize(FONT_SIZE);
	resetText.setColor(RESET_CONFIRMATION_COLOR);
	resetText.setString(RESET_CONFIRMATION_MESSAGE);
	sf::Vector2i resetTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), resetConfirmationTextRectangle);
	resetText.setPosition(resetTextCenter.x - resetConfirmationTextRectangle.getSize().x, resetTextCenter.y);
	
	sf::Text resetCountdownText;
	resetCountdownText.setFont(font);
	resetCountdownText.setCharacterSize(FONT_SIZE);
	resetCountdownText.setColor(RESET_CONFIRMATION_COLOR);
	resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_5);
	sf::Vector2i resetCountdownTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), resetConfirmationCountdownRectangle);
	resetCountdownText.setPosition(resetCountdownTextCenter.x, resetCountdownTextCenter.y);

	sf::Text resetButtonText;
	resetButtonText.setFont(font);
	resetButtonText.setCharacterSize(FONT_SIZE);
	resetButtonText.setColor(RESET_CONFIRMATION_BUTTON_TEXT_COLOR);
	resetButtonText.setString(RESET_CANCEL_BUTTON_MESSAGE);
	sf::Vector2i resetButtonTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), resetConfirmationButtonRectangle);
	resetButtonText.setPosition(resetButtonTextCenter.x - resetConfirmationButtonRectangle.getSize().x/7, resetButtonTextCenter.y);

	sf::Text robotLoadingText;
	robotLoadingText.setFont(font);
	robotLoadingText.setCharacterSize(LOADING_TEXT_SIZE);
	robotLoadingText.setColor(LOADING_TEXT_COLOR);
	robotLoadingText.setString(LOADING_MESSAGE_1);
	sf::Vector2i robotLoadingTextCenter = centerSpriteOnRect(sf::Vector2i(LOADING_TEXT_SIZE, LOADING_TEXT_SIZE), robotLoadingRectangle);
	robotLoadingText.setPosition(robotLoadingTextCenter.x - LOADING_TEXT_SIZE, robotLoadingTextCenter.y);

	sf::Text gasText;
	gasText.setFont(font);
	gasText.setCharacterSize(FONT_SIZE);
	gasText.setColor(sf::Color::White);
	gasText.setString("10");
	sf::Vector2i gasTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), gasGaugeRectangle);
	gasText.setPosition(gasTextCenter.x, gasTextCenter.y);

	sf::Text tireText;
	tireText.setFont(font);
	tireText.setCharacterSize(FONT_SIZE);
	tireText.setColor(sf::Color::White);
	tireText.setString("10");
	sf::Vector2i tireTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), tireGaugeRectangle);
	tireText.setPosition(tireTextCenter.x, tireTextCenter.y);

	sf::Text helpGasGaugeText;
	helpGasGaugeText.setFont(font);
	helpGasGaugeText.setCharacterSize(FONT_SIZE);
	helpGasGaugeText.setColor(sf::Color::Black);
	helpGasGaugeText.setString(HELP_MESSAGE_GAS_GAUGE);
	sf::Vector2i helpGasGaugeTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), gasGaugeRectangle);
	helpGasGaugeText.setPosition(helpGasGaugeTextCenter.x - 2*FONT_SIZE, helpGasGaugeTextCenter.y + ICON_SIZE.y);

	sf::Text helpTireGaugeText;
	helpTireGaugeText.setFont(font);
	helpTireGaugeText.setCharacterSize(FONT_SIZE);
	helpTireGaugeText.setColor(sf::Color::Black);
	helpTireGaugeText.setString(HELP_MESSAGE_TIRE_GAUGE);
	sf::Vector2i helpTireGaugeTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), tireGaugeRectangle);
	helpTireGaugeText.setPosition(helpTireGaugeTextCenter.x - 2*FONT_SIZE, helpTireGaugeTextCenter.y + ICON_SIZE.y);

	sf::Text helpCrewText;
	helpCrewText.setFont(font);
	helpCrewText.setCharacterSize(FONT_SIZE);
	helpCrewText.setColor(sf::Color::Black);
	helpCrewText.setString(HELP_MESSAGE_PIT_CREW);
	sf::Vector2i helpCrewTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), crewRectangle);
	helpCrewText.setPosition(helpCrewTextCenter.x - FONT_SIZE, helpCrewTextCenter.y + ICON_SIZE.y);

	sf::Text helpGasText;
	helpGasText.setFont(font);
	helpGasText.setCharacterSize(FONT_SIZE);
	helpGasText.setColor(sf::Color::Black);
	helpGasText.setString(HELP_MESSAGE_GAS_BUTTON);
	sf::Vector2i helpGasTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), gasRectangle);
	helpGasText.setPosition(helpGasTextCenter.x - ICON_SIZE.x, helpGasTextCenter.y + ICON_SIZE.y);

	sf::Text helpTireText;
	helpTireText.setFont(font);
	helpTireText.setCharacterSize(FONT_SIZE);
	helpTireText.setColor(sf::Color::Black);
	helpTireText.setString(HELP_MESSAGE_TIRE_BUTTON);
	sf::Vector2i helpTireTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), tireRectangle);
	helpTireText.setPosition(helpTireTextCenter.x - ICON_SIZE.x, helpTireTextCenter.y + ICON_SIZE.y);

	sf::Text helpHelpText;
	helpHelpText.setFont(font);
	helpHelpText.setCharacterSize(FONT_SIZE);
	helpHelpText.setColor(sf::Color::Black);
	helpHelpText.setString(HELP_MESSAGE);
	sf::Vector2i helpHelpTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), helpRectangle);
	helpHelpText.setPosition(helpHelpTextCenter.x - ICON_SIZE.x, helpHelpTextCenter.y + ICON_SIZE.y);

	sf::Text helpResetText;
	helpResetText.setFont(font);
	helpResetText.setCharacterSize(FONT_SIZE);
	helpResetText.setColor(sf::Color::Black);
	helpResetText.setString(HELP_MESSAGE_RESET_BUTTON);
	sf::Vector2i helpResetTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), handRectangle);
	helpResetText.setPosition(helpResetTextCenter.x - ICON_SIZE.x, helpResetTextCenter.y + ICON_SIZE.y);

	sf::Text leapText;
	leapText.setFont(font);
	leapText.setCharacterSize(FONT_SIZE);
	updateLeapMessage(leapText);
	
	sf::Text initializeText;
	initializeText.setFont(font);
	initializeText.setCharacterSize(FONT_SIZE*3);
	initializeText.setColor(sf::Color::Yellow);
	initializeText.setString(LOADING_MESSAGE_1);
	sf::Vector2i initializeTextCenter = centerSpriteOnRect(sf::Vector2i(FONT_SIZE, FONT_SIZE), backgroundRectangle);
	initializeText.setPosition(initializeTextCenter.x - 4*FONT_SIZE, initializeTextCenter.y);

	
	
	//Set up the serial comm port
	
	if(RS232_OpenComport(ROBOT_PORT_NUMBER, ROBOT_BAUDRATE))
	{//Error...
		//crewRectangle.setFillColor(sf::Color::Red);
		queuedSound &= SOUND_ROBOT_CONNECTION_ERROR;
	} else {
		//crewRectangle.setFillColor(sf::Color::Cyan);

		refreshRobotPosition = 1;

	}
	
	if(RS232_OpenComport(CAR_PORT_NUMBER, CAR_BAUDRATE))
	{//Error...
		leapRectangle.setFillColor(sf::Color::Red);
		queuedSound &= SOUND_CAR_CONNECTION_ERROR;
	} else {
		leapRectangle.setFillColor(sf::Color::Green);

		driveCar = 1;
	}
	
	//Set up the Leap Motion code

	// Create a sample listener and controller
	SampleListener listener;
	Controller controller;

	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	

	//Run main loop
    while (window.isOpen())
    {
		//If appropriate, set the cursor to follow the Leap
		if(leapControl)
			sf::Mouse::setPosition(sf::Vector2i(LEAP_CONTROL_X, LEAP_CONTROL_Y));

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
			//Time to close the window, for whatever reason
                window.close();

			if (event.type == sf::Event::MouseButtonPressed)
			//A mouse button has been pressed
			{	
				if (event.mouseButton.button == sf::Mouse::Left)
				//It was the left mouse button
				{
					mousePressed = 1;
				}

				if (event.mouseButton.button == sf::Mouse::Right)
				//It was the right mouse button
				{
				}
			}

			if(event.type == sf::Event::MouseMoved)
			//The cursor has moved - capture the new coordinates
			{
				mousePos = sf::Mouse::getPosition(window);
			}

			if (event.type == sf::Event::MouseButtonReleased)
			//The mouse button has been released
			{
				mousePressed = 0;
			} 

			if (event.type == sf::Event::MouseEntered)
			//The cursor has entered the client window
			{
			}

			if (event.type == sf::Event::MouseLeft)
			//The cursor has left the client window
			{
			}

			if (event.type == sf::Event::KeyPressed)
			//A key has been pressed on the keyboard
			{
				if(event.key.code == sf::Keyboard::Escape)
				//Close the window (Escape!)
					window.close();

				if(event.key.code == sf::Keyboard::Return)
				//Switch between Mouse and Leap control
					leapControl = !leapControl;

				if(event.key.code == sf::Keyboard::Delete)
				//Close the Robot COM Port
					RS232_CloseComport(ROBOT_PORT_NUMBER);
			}
        }

		

		//Adjust icons based on mouse position

		//Gas icon
		if(isMouseInRect(gasRectangle) && selectedIcon.getPosition() != gasRectangle.getPosition() && !robotArmActive)
		{
			//Set the selected icon
			selectedIcon = gasRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(gasSprite.getScale().x <= 1 && gasSprite.getScale().y <= 1){
				gasSprite.move(-3.2,-3.2);
				gasSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(gasRectangle) && selectedIcon.getPosition() == gasRectangle.getPosition() && !robotArmActive) 
		{	
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(gasSprite.getScale().x > 1 && gasSprite.getScale().y > 1){
					gasSprite.move(3.2,3.2);
					gasSprite.setScale(0.99,0.99);
				}
				gasSprite.setColor(CLICK_COLOR);
				
				if(!robotArmActive && !loadingActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							loadingActive = 1;
							mousePressed = 0;
							clickClock.restart();
							robotStopLoadingClock.restart();
							robotLoadingClock.restart();
									
							refreshRobotPosition = 1;
							gasGame = 1;


						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							loadingActive = 1;
							mousePressed = 0;
							clickClock.restart();
							robotStopLoadingClock.restart();
							robotLoadingClock.restart();
							
							refreshRobotPosition = 1;
							gasGame = 1;
		
						}
						else;
					}
				}
			} else {
				if(gasSprite.getScale().x <= 1 && gasSprite.getScale().y <= 1){
					gasSprite.move(-3.2,-3.2);
					gasSprite.setScale(1.1,1.1);
				}
				gasSprite.setColor(REST_COLOR);
			}
		}
		else 
		{
			if(gasSprite.getScale().x > 1 && gasSprite.getScale().y > 1){
				gasSprite.move(3.2,3.2);
				gasSprite.setScale(0.99,0.99);
			}
			gasSprite.setColor(REST_COLOR);
		}

		//Tire icon
		if(isMouseInRect(tireRectangle) && selectedIcon.getPosition() != tireRectangle.getPosition() && !robotArmActive)
		{
			//Set the selected icon
			selectedIcon = tireRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(tireSprite.getScale().x <= 1 && tireSprite.getScale().y <= 1){
				tireSprite.move(-3.2,-3.2);
				tireSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(tireRectangle) && selectedIcon.getPosition() == tireRectangle.getPosition() && !robotArmActive) 
		{	
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(tireSprite.getScale().x > 1 && tireSprite.getScale().y > 1){
					tireSprite.move(3.2,3.2);
					tireSprite.setScale(0.99,0.99);
				}
				tireSprite.setColor(CLICK_COLOR);
				
				if(!robotArmActive && !loadingActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							loadingActive = 1;
							mousePressed = 0;
							clickClock.restart();
							robotStopLoadingClock.restart();
							robotLoadingClock.restart();
							
							refreshRobotPosition = 1;
							tireGame = 1;

						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							loadingActive = 1;
							mousePressed = 0;
							clickClock.restart();
							robotStopLoadingClock.restart();
							robotLoadingClock.restart();
							
							refreshRobotPosition = 1;
							tireGame = 1;

						}
						else;
					}
				}
			} else {
				if(tireSprite.getScale().x <= 1 && tireSprite.getScale().y <= 1){
					tireSprite.move(-3.2,-3.2);
					tireSprite.setScale(1.1,1.1);
				}
				tireSprite.setColor(REST_COLOR);
			}
		}
		else 
		{
			if(tireSprite.getScale().x > 1 && tireSprite.getScale().y > 1){
				tireSprite.move(3.2,3.2);
				tireSprite.setScale(0.99,0.99);
			}
			tireSprite.setColor(REST_COLOR);
		}

		
		//Hand icon
		if(isMouseInRect(handRectangle) && selectedIcon.getPosition() != handRectangle.getPosition() && !robotArmActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(handSprite.getScale().x <= 1 && handSprite.getScale().y <= 1){
				handSprite.move(-3.2,-3.2);
				handSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(handRectangle) && selectedIcon.getPosition() == handRectangle.getPosition() && !robotArmActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(handSprite.getScale().x > 1 && handSprite.getScale().y > 1){
					handSprite.move(3.2,3.2);
					handSprite.setScale(0.99,0.99);
				}
				handSprite.setColor(CLICK_COLOR);

				
				if(!resetActive && !loadingActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							resetActive = 1;
							mousePressed = 0;
							clickClock.restart();
							resetClock.restart();
						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							resetActive = 1;
							mousePressed = 0;
							clickClock.restart();
							resetClock.restart();
						}
						else;
					}
				}

			} else {
				if(handSprite.getScale().x <= 1 && handSprite.getScale().y <= 1){
					handSprite.move(-3.2,-3.2);
					handSprite.setScale(1.1,1.1);
				}
				handSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(handSprite.getScale().x > 1 && handSprite.getScale().y > 1){
				handSprite.move(3.2,3.2);
				handSprite.setScale(0.99,0.99);
			}
			handSprite.setColor(REST_COLOR);
		}

		
		//Help icon
		if(isMouseInRect(helpRectangle) && selectedIcon.getPosition() != helpRectangle.getPosition() && !robotArmActive)
		{
			//Set the selected icon
			selectedIcon = helpRectangle;

			clickClock.restart();

			if(leapControl)
				mousePressed = 0;

			if(helpSprite.getScale().x <= 1 && helpSprite.getScale().y <= 1){
				helpSprite.move(-3.2,-3.2);
				helpSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(helpRectangle) && selectedIcon.getPosition() == helpRectangle.getPosition() && !robotArmActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(helpSprite.getScale().x > 1 && helpSprite.getScale().y > 1){
					helpSprite.move(3.2,3.2);
					helpSprite.setScale(0.99,0.99);
				}
				helpSprite.setColor(CLICK_COLOR);

				if(!loadingActive && !robotArmActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							helpActive = !helpActive;
							mousePressed = 0;
							clickClock.restart();
						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							helpActive = !helpActive;
							mousePressed = 0;
							clickClock.restart();
						}
						else;
					}
				}
			} else {
				if(helpSprite.getScale().x <= 1 && helpSprite.getScale().y <= 1){
					helpSprite.move(-3.2,-3.2);
					helpSprite.setScale(1.1,1.1);
				}
				helpSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(helpSprite.getScale().x > 1 && helpSprite.getScale().y > 1){
				helpSprite.move(3.2,3.2);
				helpSprite.setScale(0.99,0.99);
			}
			helpSprite.setColor(REST_COLOR);
		}

		/*
		//Leap icon
		if(isMouseInRect(leapRectangle) && selectedIcon.getPosition() != leapRectangle.getPosition())
		{
			//Set the selected icon
			selectedIcon = leapRectangle;

			if(leapSprite.getScale().x <= 1 && leapSprite.getScale().y <= 1){
				leapSprite.move(-3.2,-3.2);
				leapSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(leapRectangle) && selectedIcon.getPosition() == leapRectangle.getPosition()) 
		{	
			
			if(mousePressed){
				if(leapSprite.getScale().x > 1 && leapSprite.getScale().y > 1){
					leapSprite.move(3.2,3.2);
					leapSprite.setScale(0.99,0.99);
				}
				leapSprite.setColor(CLICK_COLOR);
			} else {
				if(leapSprite.getScale().x <= 1 && leapSprite.getScale().y <= 1){
					leapSprite.move(-3.2,-3.2);
					leapSprite.setScale(1.1,1.1);
				}
				leapSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(leapSprite.getScale().x > 1 && leapSprite.getScale().y > 1){
				leapSprite.move(3.2,3.2);
				leapSprite.setScale(0.99,0.99);
			}
		}

		
		//Tire Gauge icon
		if(isMouseInRect(tireGaugeRectangle) && selectedIcon.getPosition() != tireGaugeRectangle.getPosition())
		{
			//Set the selected icon
			selectedIcon = tireGaugeRectangle;

			if(tireGaugeSprite.getScale().x <= 1 && tireGaugeSprite.getScale().y <= 1){
				tireGaugeSprite.move(-3.2,-3.2);
				tireGaugeSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(tireGaugeRectangle) && selectedIcon.getPosition() == tireGaugeRectangle.getPosition()) 
		{	
			
			if(mousePressed){
				if(tireGaugeSprite.getScale().x > 1 && tireGaugeSprite.getScale().y > 1){
					tireGaugeSprite.move(3.2,3.2);
					tireGaugeSprite.setScale(0.99,0.99);
				}
				tireGaugeSprite.setColor(CLICK_COLOR);
			} else {
				if(tireGaugeSprite.getScale().x <= 1 && tireGaugeSprite.getScale().y <= 1){
					tireGaugeSprite.move(-3.2,-3.2);
					tireGaugeSprite.setScale(1.1,1.1);
				}
				tireGaugeSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(tireGaugeSprite.getScale().x > 1 && tireGaugeSprite.getScale().y > 1){
				tireGaugeSprite.move(3.2,3.2);
				tireGaugeSprite.setScale(0.99,0.99);
			}
		}

		//Gas Gauge icon
		if(isMouseInRect(gasGaugeRectangle) && selectedIcon.getPosition() != gasGaugeRectangle.getPosition())
		{
			//Set the selected icon
			selectedIcon = gasGaugeRectangle;

			if(gasGaugeSprite.getScale().x <= 1 && gasGaugeSprite.getScale().y <= 1){
				gasGaugeSprite.move(-3.2,-3.2);
				gasGaugeSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(gasGaugeRectangle) && selectedIcon.getPosition() == gasGaugeRectangle.getPosition()) 
		{	
			
			if(mousePressed){
				if(gasGaugeSprite.getScale().x > 1 && gasGaugeSprite.getScale().y > 1){
					gasGaugeSprite.move(3.2,3.2);
					gasGaugeSprite.setScale(0.99,0.99);
				}
				gasGaugeSprite.setColor(CLICK_COLOR);
			} else {
				if(gasGaugeSprite.getScale().x <= 1 && gasGaugeSprite.getScale().y <= 1){
					gasGaugeSprite.move(-3.2,-3.2);
					gasGaugeSprite.setScale(1.1,1.1);
				}
				gasGaugeSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(gasGaugeSprite.getScale().x > 1 && gasGaugeSprite.getScale().y > 1){
				gasGaugeSprite.move(3.2,3.2);
				gasGaugeSprite.setScale(0.99,0.99);
			}
		}
		
		//Crew icon
		if(isMouseInRect(crewRectangle) && selectedIcon.getPosition() != crewRectangle.getPosition())
		{
			//Set the selected icon
			selectedIcon = crewRectangle;

			if(crewSprite.getScale().x <= 1 && crewSprite.getScale().y <= 1){
				crewSprite.move(-3.2,-3.2);
				crewSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(crewRectangle) && selectedIcon.getPosition() == crewRectangle.getPosition()) 
		{	
			
			if(mousePressed){
				if(crewSprite.getScale().x > 1 && crewSprite.getScale().y > 1){
					crewSprite.move(3.2,3.2);
					crewSprite.setScale(0.99,0.99);
				}
				crewSprite.setColor(CLICK_COLOR);
			} else {
				if(crewSprite.getScale().x <= 1 && crewSprite.getScale().y <= 1){
					crewSprite.move(-3.2,-3.2);
					crewSprite.setScale(1.1,1.1);
				}
				crewSprite.setColor(REST_COLOR);
			}
		}
		else 
		{

			if(crewSprite.getScale().x > 1 && crewSprite.getScale().y > 1){
				crewSprite.move(3.2,3.2);
				crewSprite.setScale(0.99,0.99);
			}
		}
		*/
		
		//Up Left icon
		if(isMouseInRect(robotUpLeftRectangle) && selectedIcon.getPosition() != robotUpLeftRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = robotUpLeftRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(robotUpLeftSprite.getScale().x <= 1 && robotUpLeftSprite.getScale().y <= 1){
				robotUpLeftSprite.move(-3.2,-3.2);
				robotUpLeftSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotUpLeftRectangle) && selectedIcon.getPosition() == robotUpLeftRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotUpLeftSprite.getScale().x > 1 && robotUpLeftSprite.getScale().y > 1){
					robotUpLeftSprite.move(3.2,3.2);
					robotUpLeftSprite.setScale(0.99,0.99);
				}
				robotUpLeftSprite.setColor(CLICK_COLOR);
				robotUpLeftRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_LEFT_COMMAND;
				vertCommand = VERTICAL_UP_COMMAND;

				refreshRobotPosition = 1;


			} else {
				if(robotUpLeftSprite.getScale().x <= 1 && robotUpLeftSprite.getScale().y <= 1){
					robotUpLeftSprite.move(-3.2,-3.2);
					robotUpLeftSprite.setScale(1.1,1.1);
				}
				robotUpLeftSprite.setColor(REST_COLOR);
				robotUpLeftRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotUpLeftSprite.getScale().x > 1 && robotUpLeftSprite.getScale().y > 1){
				robotUpLeftSprite.move(3.2,3.2);
				robotUpLeftSprite.setScale(0.99,0.99);
			}
			robotUpLeftSprite.setColor(REST_COLOR);
			robotUpLeftRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Up icon
		if(isMouseInRect(robotUpRectangle) && selectedIcon.getPosition() != robotUpRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = robotUpRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(robotUpSprite.getScale().x <= 1 && robotUpSprite.getScale().y <= 1){
				robotUpSprite.move(-3.2,-3.2);
				robotUpSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotUpRectangle) && selectedIcon.getPosition() == robotUpRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotUpSprite.getScale().x > 1 && robotUpSprite.getScale().y > 1){
					robotUpSprite.move(3.2,3.2);
					robotUpSprite.setScale(0.99,0.99);
				}
				robotUpSprite.setColor(CLICK_COLOR);
				robotUpRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_NEUTRAL_COMMAND;
				vertCommand = VERTICAL_UP_COMMAND;
				
				refreshRobotPosition = 1;


			} else {
				if(robotUpSprite.getScale().x <= 1 && robotUpSprite.getScale().y <= 1){
					robotUpSprite.move(-3.2,-3.2);
					robotUpSprite.setScale(1.1,1.1);
				}
				robotUpSprite.setColor(REST_COLOR);
				robotUpRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotUpSprite.getScale().x > 1 && robotUpSprite.getScale().y > 1){
				robotUpSprite.move(3.2,3.2);
				robotUpSprite.setScale(0.99,0.99);
			}
			robotUpSprite.setColor(REST_COLOR);
			robotUpRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Up Right icon
		if(isMouseInRect(robotUpRightRectangle) && selectedIcon.getPosition() != robotUpRightRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotUpRightRectangle;

			if(robotUpRightSprite.getScale().x <= 1 && robotUpRightSprite.getScale().y <= 1){
				robotUpRightSprite.move(-3.2,-3.2);
				robotUpRightSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotUpRightRectangle) && selectedIcon.getPosition() == robotUpRightRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotUpRightSprite.getScale().x > 1 && robotUpRightSprite.getScale().y > 1){
					robotUpRightSprite.move(3.2,3.2);
					robotUpRightSprite.setScale(0.99,0.99);
				}
				robotUpRightSprite.setColor(CLICK_COLOR);
				robotUpRightRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_RIGHT_COMMAND;
				vertCommand = VERTICAL_UP_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotUpRightSprite.getScale().x <= 1 && robotUpRightSprite.getScale().y <= 1){
					robotUpRightSprite.move(-3.2,-3.2);
					robotUpRightSprite.setScale(1.1,1.1);
				}
				robotUpRightSprite.setColor(REST_COLOR);
				robotUpRightRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotUpRightSprite.getScale().x > 1 && robotUpRightSprite.getScale().y > 1){
				robotUpRightSprite.move(3.2,3.2);
				robotUpRightSprite.setScale(0.99,0.99);
			}
			robotUpRightSprite.setColor(REST_COLOR);
			robotUpRightRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Left icon
		if(isMouseInRect(robotLeftRectangle) && selectedIcon.getPosition() != robotLeftRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotLeftRectangle;

			if(robotLeftSprite.getScale().x <= 1 && robotLeftSprite.getScale().y <= 1){
				robotLeftSprite.move(-3.2,-3.2);
				robotLeftSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotLeftRectangle) && selectedIcon.getPosition() == robotLeftRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotLeftSprite.getScale().x > 1 && robotLeftSprite.getScale().y > 1){
					robotLeftSprite.move(3.2,3.2);
					robotLeftSprite.setScale(0.99,0.99);
				}
				robotLeftSprite.setColor(CLICK_COLOR);
				robotLeftRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_LEFT_COMMAND;
				vertCommand = VERTICAL_NEUTRAL_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotLeftSprite.getScale().x <= 1 && robotLeftSprite.getScale().y <= 1){
					robotLeftSprite.move(-3.2,-3.2);
					robotLeftSprite.setScale(1.1,1.1);
				}
				robotLeftSprite.setColor(REST_COLOR);
				robotLeftRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotLeftSprite.getScale().x > 1 && robotLeftSprite.getScale().y > 1){
				robotLeftSprite.move(3.2,3.2);
				robotLeftSprite.setScale(0.99,0.99);
			}
			robotLeftSprite.setColor(REST_COLOR);
			robotLeftRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Stop icon
		if(isMouseInRect(robotStopRectangle) && selectedIcon.getPosition() != robotStopRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotStopRectangle;

			if(robotStopSprite.getScale().x <= 1 && robotStopSprite.getScale().y <= 1){
				robotStopSprite.move(-3.2,-3.2);
				robotStopSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotStopRectangle) && selectedIcon.getPosition() == robotStopRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotStopSprite.getScale().x > 1 && robotStopSprite.getScale().y > 1){
					robotStopSprite.move(3.2,3.2);
					robotStopSprite.setScale(0.99,0.99);
				}
				robotStopSprite.setColor(CLICK_COLOR);
				robotStopRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_NEUTRAL_COMMAND;
				vertCommand = VERTICAL_NEUTRAL_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotStopSprite.getScale().x <= 1 && robotStopSprite.getScale().y <= 1){
					robotStopSprite.move(-3.2,-3.2);
					robotStopSprite.setScale(1.1,1.1);
				}
				robotStopSprite.setColor(REST_COLOR);
				robotStopRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotStopSprite.getScale().x > 1 && robotStopSprite.getScale().y > 1){
				robotStopSprite.move(3.2,3.2);
				robotStopSprite.setScale(0.99,0.99);
			}
			robotStopSprite.setColor(REST_COLOR);
			robotStopRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Right icon
		if(isMouseInRect(robotRightRectangle) && selectedIcon.getPosition() != robotRightRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotRightRectangle;

			if(robotRightSprite.getScale().x <= 1 && robotRightSprite.getScale().y <= 1){
				robotRightSprite.move(-3.2,-3.2);
				robotRightSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotRightRectangle) && selectedIcon.getPosition() == robotRightRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotRightSprite.getScale().x > 1 && robotRightSprite.getScale().y > 1){
					robotRightSprite.move(3.2,3.2);
					robotRightSprite.setScale(0.99,0.99);
				}
				robotRightSprite.setColor(CLICK_COLOR);
				robotRightRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_RIGHT_COMMAND;
				vertCommand = VERTICAL_NEUTRAL_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotRightSprite.getScale().x <= 1 && robotRightSprite.getScale().y <= 1){
					robotRightSprite.move(-3.2,-3.2);
					robotRightSprite.setScale(1.1,1.1);
				}
				robotRightSprite.setColor(REST_COLOR);
				robotRightRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotRightSprite.getScale().x > 1 && robotRightSprite.getScale().y > 1){
				robotRightSprite.move(3.2,3.2);
				robotRightSprite.setScale(0.99,0.99);
			}
			robotRightSprite.setColor(REST_COLOR);
			robotRightRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Down Left icon
		if(isMouseInRect(robotDownLeftRectangle) && selectedIcon.getPosition() != robotDownLeftRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotDownLeftRectangle;

			if(robotDownLeftSprite.getScale().x <= 1 && robotDownLeftSprite.getScale().y <= 1){
				robotDownLeftSprite.move(-3.2,-3.2);
				robotDownLeftSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotDownLeftRectangle) && selectedIcon.getPosition() == robotDownLeftRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotDownLeftSprite.getScale().x > 1 && robotDownLeftSprite.getScale().y > 1){
					robotDownLeftSprite.move(3.2,3.2);
					robotDownLeftSprite.setScale(0.99,0.99);
				}
				robotDownLeftSprite.setColor(CLICK_COLOR);
				robotDownLeftRectangle.setFillColor(CONTROL_CLICK_COLOR);
				
				horzCommand = HORIZONTAL_LEFT_COMMAND;
				vertCommand = VERTICAL_DOWN_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotDownLeftSprite.getScale().x <= 1 && robotDownLeftSprite.getScale().y <= 1){
					robotDownLeftSprite.move(-3.2,-3.2);
					robotDownLeftSprite.setScale(1.1,1.1);
				}
				robotDownLeftSprite.setColor(REST_COLOR);
				robotDownLeftRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotDownLeftSprite.getScale().x > 1 && robotDownLeftSprite.getScale().y > 1){
				robotDownLeftSprite.move(3.2,3.2);
				robotDownLeftSprite.setScale(0.99,0.99);
			}
			robotDownLeftSprite.setColor(REST_COLOR);
			robotDownLeftRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Down icon
		if(isMouseInRect(robotDownRectangle) && selectedIcon.getPosition() != robotDownRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotDownRectangle;

			if(robotDownSprite.getScale().x <= 1 && robotDownSprite.getScale().y <= 1){
				robotDownSprite.move(-3.2,-3.2);
				robotDownSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotDownRectangle) && selectedIcon.getPosition() == robotDownRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotDownSprite.getScale().x > 1 && robotDownSprite.getScale().y > 1){
					robotDownSprite.move(3.2,3.2);
					robotDownSprite.setScale(0.99,0.99);
				}
				robotDownSprite.setColor(CLICK_COLOR);
				robotDownRectangle.setFillColor(CONTROL_CLICK_COLOR);
				
				horzCommand = HORIZONTAL_NEUTRAL_COMMAND;
				vertCommand = VERTICAL_DOWN_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotDownSprite.getScale().x <= 1 && robotDownSprite.getScale().y <= 1){
					robotDownSprite.move(-3.2,-3.2);
					robotDownSprite.setScale(1.1,1.1);
				}
				robotDownSprite.setColor(REST_COLOR);
				robotDownRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotDownSprite.getScale().x > 1 && robotDownSprite.getScale().y > 1){
				robotDownSprite.move(3.2,3.2);
				robotDownSprite.setScale(0.99,0.99);
			}
			robotDownSprite.setColor(REST_COLOR);
			robotDownRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Down Right icon
		if(isMouseInRect(robotDownRightRectangle) && selectedIcon.getPosition() != robotDownRightRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotDownRightRectangle;

			if(robotDownRightSprite.getScale().x <= 1 && robotDownRightSprite.getScale().y <= 1){
				robotDownRightSprite.move(-3.2,-3.2);
				robotDownRightSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotDownRightRectangle) && selectedIcon.getPosition() == robotDownRightRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotDownRightSprite.getScale().x > 1 && robotDownRightSprite.getScale().y > 1){
					robotDownRightSprite.move(3.2,3.2);
					robotDownRightSprite.setScale(0.99,0.99);
				}
				robotDownRightSprite.setColor(CLICK_COLOR);
				robotDownRightRectangle.setFillColor(CONTROL_CLICK_COLOR);

				horzCommand = HORIZONTAL_RIGHT_COMMAND;
				vertCommand = VERTICAL_DOWN_COMMAND;
				
				refreshRobotPosition = 1;

			} else {
				if(robotDownRightSprite.getScale().x <= 1 && robotDownRightSprite.getScale().y <= 1){
					robotDownRightSprite.move(-3.2,-3.2);
					robotDownRightSprite.setScale(1.1,1.1);
				}
				robotDownRightSprite.setColor(REST_COLOR);
				robotDownRightRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotDownRightSprite.getScale().x > 1 && robotDownRightSprite.getScale().y > 1){
				robotDownRightSprite.move(3.2,3.2);
				robotDownRightSprite.setScale(0.99,0.99);
			}
				robotDownRightSprite.setColor(REST_COLOR);
				robotDownRightRectangle.setFillColor(CONTROL_REST_COLOR);
		}
		
		//Exit icon
		if(isMouseInRect(robotExitRectangle) && selectedIcon.getPosition() != robotExitRectangle.getPosition() && robotArmActive && !loadingActive)
		{
			//Set the selected icon
			selectedIcon = handRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			selectedIcon = robotExitRectangle;

			if(robotExitSprite.getScale().x <= 1 && robotExitSprite.getScale().y <= 1){
				robotExitSprite.move(-3.2,-3.2);
				robotExitSprite.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(robotExitRectangle) && selectedIcon.getPosition() == robotExitRectangle.getPosition() && robotArmActive && !loadingActive) 
		{	
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(robotExitSprite.getScale().x > 1 && robotExitSprite.getScale().y > 1){
					robotExitSprite.move(3.2,3.2);
					robotExitSprite.setScale(0.99,0.99);
				}
				robotExitSprite.setColor(CLICK_COLOR);
				robotExitRectangle.setFillColor(CONTROL_CLICK_COLOR);

				if(robotArmActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							robotArmActive = 0;
							mousePressed = 0;
							clickClock.restart();
				
							refreshRobotPosition = 1;
							gasGame = 0;
							tireGame = 0;
						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							robotArmActive = 0;
							mousePressed = 0;
							clickClock.restart();
				
							refreshRobotPosition = 1;
							gasGame = 0;
							tireGame = 0;
						}
						else;
					}
				}
			} else {
				if(robotExitSprite.getScale().x <= 1 && robotExitSprite.getScale().y <= 1){
					robotExitSprite.move(-3.2,-3.2);
					robotExitSprite.setScale(1.1,1.1);
				}
				robotExitSprite.setColor(REST_COLOR);
				robotExitRectangle.setFillColor(CONTROL_REST_COLOR);
			}
		}
		else 
		{

			if(robotExitSprite.getScale().x > 1 && robotExitSprite.getScale().y > 1){
				robotExitSprite.move(3.2,3.2);
				robotExitSprite.setScale(0.99,0.99);
			}
			robotExitSprite.setColor(REST_COLOR);
			robotExitRectangle.setFillColor(CONTROL_REST_COLOR);
		}

		//Cancel Reset icon
		if(isMouseInRect(resetConfirmationButtonRectangle) && selectedIcon.getPosition() != resetConfirmationButtonRectangle.getPosition() && resetActive)
		{
			//Set the selected icon
			selectedIcon = resetConfirmationButtonRectangle;

			clickClock.restart();
			
			if(leapControl)
				mousePressed = 0;

			if(resetButtonText.getScale().x <= 1 && resetButtonText.getScale().y <= 1){
				resetButtonText.move(-3.2,-3.2);
				resetButtonText.setScale(1.1,1.1);
			}

		} 
		else if(isMouseInRect(resetConfirmationButtonRectangle) && selectedIcon.getPosition() == resetConfirmationButtonRectangle.getPosition() && resetActive) 
		{
			
			if(leapControl){
				if(clickClock.getElapsedTime().asMilliseconds() > HOLD_TIME_TO_CLICK)
					mousePressed = 1;
			}

			if(mousePressed){
				if(resetButtonText.getScale().x > 1 && resetButtonText.getScale().y > 1){
					resetButtonText.move(3.2,3.2);
					resetButtonText.setScale(0.99,0.99);
				}
				resetButtonText.setColor(RESET_CONFIRMATION_BUTTON_TEXT_COLOR);
				resetConfirmationButtonRectangle.setFillColor(RESET_BUTTON_CANCEL_CLICKED_COLOR);

				if(resetActive){
					if(leapControl){
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS)
						{
							resetActive = 0;
							mousePressed = 0;
							clickClock.restart();
						}
						else;
					} else {
						if(changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS + TIME_TO_DELAY_MENUS_FUDGE_FACTOR - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
							changeMenuClock.restart();
						else if (changeMenuClock.getElapsedTime().asMilliseconds() > TIME_TO_DELAY_MENUS - TIME_TO_DELAY_MENUS_MOUSE_REDUCTION)
						{
							resetActive = 0;
							mousePressed = 0;
							clickClock.restart();
						}
						else;
					}
				}

			} else {
				if(resetButtonText.getScale().x <= 1 && resetButtonText.getScale().y <= 1){
					resetButtonText.move(-3.2,-3.2);
					resetButtonText.setScale(1.1,1.1);
				}
				resetButtonText.setColor(RESET_CONFIRMATION_BUTTON_TEXT_COLOR);
				resetConfirmationButtonRectangle.setFillColor(RESET_CONFIRMATION_COLOR);
			}
		}
		else 
		{

			if(resetButtonText.getScale().x > 1 && resetButtonText.getScale().y > 1){
				resetButtonText.move(3.2,3.2);
				resetButtonText.setScale(0.99,0.99);
			}
			resetButtonText.setColor(RESET_CONFIRMATION_BUTTON_TEXT_COLOR);
			resetConfirmationButtonRectangle.setFillColor(RESET_CONFIRMATION_COLOR);
		}

		if(!isMouseInRect(gasRectangle)
			&& !isMouseInRect(tireRectangle)
			&& !isMouseInRect(handRectangle)
			&& !isMouseInRect(helpRectangle)
//			&& !isMouseInRect(leapRectangle)
//			&& !isMouseInRect(crewRectangle)
//			&& !isMouseInRect(gasGaugeRectangle)
//			&& !isMouseInRect(tireGaugeRectangle)
			&& !isMouseInRect(robotUpLeftRectangle)
			&& !isMouseInRect(robotUpRectangle)
			&& !isMouseInRect(robotUpRightRectangle)
			&& !isMouseInRect(robotLeftRectangle)
			&& !isMouseInRect(robotStopRectangle)
			&& !isMouseInRect(robotRightRectangle)
			&& !isMouseInRect(robotDownLeftRectangle)
			&& !isMouseInRect(robotDownRectangle)
			&& !isMouseInRect(robotDownRightRectangle)
			&& !isMouseInRect(robotExitRectangle)
			&& !isMouseInRect(resetConfirmationButtonRectangle)
			)
		{
			//Reset the mousePressed variable
			mousePressed = 0;

			//Reset the selected icon
			selectedIcon = backgroundRectangle;
		}

		
		//Control the timing for the reset
		if(resetActive && resetClock.getElapsedTime().asMilliseconds() > TIME_TO_RESET)
		{
			resetGame();
		}
		else if(resetActive && resetClock.getElapsedTime().asMilliseconds() > 5*TIME_TO_RESET/6)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_0);
		}
		else if(resetActive && resetClock.getElapsedTime().asMilliseconds() > 2*TIME_TO_RESET/3)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_1);
		}
		else if(resetActive && resetClock.getElapsedTime().asMilliseconds() > TIME_TO_RESET/2)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_2);
		}else if(resetActive && resetClock.getElapsedTime().asMilliseconds() > TIME_TO_RESET/3)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_3);
		}
		else if(resetActive && resetClock.getElapsedTime().asMilliseconds() > TIME_TO_RESET/6)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_4);
		}
		else if(resetActive && resetClock.getElapsedTime().asMilliseconds() <= TIME_TO_RESET/6)
		{
			resetCountdownText.setString(RESET_COUNTDOWN_MESSAGE_5);
		}
		else;

		//Control the timing for the robot-loading screen
		if(loadingActive && robotLoadingClock.getElapsedTime().asMilliseconds() < TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			if(gasReset)
				robotLoadingText.setString(GAS_WAIT_MESSAGE_1);
			else if(tireReset)
				robotLoadingText.setString(TIRE_WAIT_MESSAGE_1);
			else;
		}
		else if(loadingActive && robotLoadingClock.getElapsedTime().asMilliseconds() < 2*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			if(gasReset)
				robotLoadingText.setString(GAS_WAIT_MESSAGE_2);
			else if(tireReset)
				robotLoadingText.setString(TIRE_WAIT_MESSAGE_2);
			else;
		}
		else if(loadingActive && robotLoadingClock.getElapsedTime().asMilliseconds() < 3*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			if(gasReset)
				robotLoadingText.setString(GAS_WAIT_MESSAGE_3);
			else if(tireReset)
				robotLoadingText.setString(TIRE_WAIT_MESSAGE_3);
			else;
		}
		else if(loadingActive && robotLoadingClock.getElapsedTime().asMilliseconds() > 3*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			robotLoadingClock.restart();
		}
		else;

		//Control the timing for the test loading clock
		if(loadingActive && robotStopLoadingClock.getElapsedTime().asMilliseconds() > TIME_TO_STOP_LOADING)
		{
			loadingActive = 0;
			robotArmActive = 1;
			driveCar = 0;
		}
		else;
		
		//Control the driving of the car
		if(driveCar ^ carIsDriving){
			
			carIsDriving = driveCar;
			
			unsigned char writeToCar = driveCar?CAR_DRIVE:CAR_STOP;
			RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
			writeToCar = NEWLINE;
			RS232_SendByte(CAR_PORT_NUMBER, writeToCar);
			
		}

		//Update Game Mechanics
		if(!initialLoad){
			if(stateClock.getElapsedTime().asMilliseconds() > TIME_TO_UPDATE_STATES){
				switch(carCommState){
				case STATE_UPDATE_GAS_VALUE:
					updateGasValue();
					carCommState = STATE_UPDATE_TIRE_VALUE;
					break;
				case STATE_UPDATE_TIRE_VALUE:
					updateTireValue();
					carCommState = STATE_UPDATE_MOTOR_SPEED;
					break;
				case STATE_UPDATE_MOTOR_SPEED:
					updateMotorSpeed();
					carCommState = STATE_UPDATE_GAS_LEDS;
					break;
				case STATE_UPDATE_GAS_LEDS:
					updateGasLEDs();
					carCommState = STATE_UPDATE_TIRE_LEDS;
					break;
				case STATE_UPDATE_TIRE_LEDS:
					updateTireLEDs();
					carCommState = STATE_UPDATE_GAS_VALUE;
					break;
				}
				stateClock.restart();
			}
		}
		

		//Update Sound State
		int soundError = updateSound();
		if(soundError != 0){
			return soundError;
		}


		//Control the timing for the gas-level clock
		if(gasUpdate && gasValue == 10){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_10.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("10");
		}
		else if(gasUpdate && gasValue == 9){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_09.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("9");
		}
		else if(gasUpdate && gasValue == 8){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_08.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("8");
		}
		else if(gasUpdate && gasValue == 7){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_07.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("7");
		}
		else if(gasUpdate && gasValue == 6){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_06.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("6");
		}
		else if(gasUpdate && gasValue == 5){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_05.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("5");
		}
		else if(gasUpdate && gasValue == 4){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_04.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("4");
		}
		else if(gasUpdate && gasValue == 3){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_03.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("3");
		}
		else if(gasUpdate && gasValue == 2){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_02.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("2");
		}
		else if(gasUpdate && gasValue == 1){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_01.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("1");
		}
		else if(gasUpdate && gasValue == 0){
			//Set the tire icon texture and sprite
			if (!gasGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\gas_station-64_00.png"))
			{}// error...

			gasGaugeSprite.setTexture(gasGaugeTexture);
			gasUpdate = 0;
			//gasText.setString("0");
		}
		else;


		//Control the timing for the tire-level clock
		if(tireUpdate && tireValue == 10){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_10.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("10");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 9){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_09.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("9");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 8){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_08.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("8");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 7){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_07.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("7");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 6){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_06.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("6");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 5){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_05.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("5");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 4){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_04.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("4");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 3){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_03.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("3");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 2){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_02.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("2");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 1){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_01.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("1");
			tireUpdate = 0;
		}
		else if(tireUpdate && tireValue == 0){
			//Set the tire icon texture and sprite
			if (!tireGaugeTexture.loadFromFile("C:\\Users\\mccullwc\\Pictures\\School Related\\ROBO 420\\tire-64_00.png"))
			{}// error...

			tireGaugeSprite.setTexture(tireGaugeTexture);
			//tireText.setString("0");
			tireUpdate = 0;
		}
		else;


		leapSprite.setColor(leapColor);
	

		if(robotArmActive && refreshRobotPosition && !initialLoad){
			
			refreshRobotPosition = 0;

			//refreshClock.restart();
			//while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			unsigned char writeToRobot = ROBOT_ARM_MOVE_PRECURSOR;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = SPACE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = horzCommand;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = SPACE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = vertCommand;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = NEWLINE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			refreshClock.restart();
			while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			
		} else if(loadingActive && refreshRobotPosition && !initialLoad){
							
			refreshRobotPosition = 0;

			//refreshClock.restart();
			//while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			unsigned char writeToRobot = ROBOT_ARM_READY_POSITION_PRECURSOR;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = SPACE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = ROBOT_ARM_READY_POSITION;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = NEWLINE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			refreshClock.restart();
			while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			
		} else if(refreshRobotPosition && !initialLoad){
			
			refreshRobotPosition = 0;

			//refreshClock.restart();
			//while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			unsigned char writeToRobot = ROBOT_ARM_HOME_POSITION_PRECURSOR;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = SPACE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = ROBOT_ARM_HOME_POSITION;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			writeToRobot = NEWLINE;
			RS232_SendByte(ROBOT_PORT_NUMBER, writeToRobot);
			refreshClock.restart();
			while(refreshClock.getElapsedTime().asMilliseconds() < TIME_TO_DELAY_REFRESH);
			
		} else;


		//Control the timing for the robot-loading screen
		if(initialLoad && robotLoadingClock.getElapsedTime().asMilliseconds() < TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			initializeText.setString(LOADING_MESSAGE_1);
		}
		else if(initialLoad && robotLoadingClock.getElapsedTime().asMilliseconds() < 2*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			initializeText.setString(LOADING_MESSAGE_2);
		}
		else if(initialLoad && robotLoadingClock.getElapsedTime().asMilliseconds() < 3*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			initializeText.setString(LOADING_MESSAGE_3);
		}
		else if(initialLoad && robotLoadingClock.getElapsedTime().asMilliseconds() > 3*TIME_TO_CHANGE_LOADING_MESSAGE)
		{
			robotLoadingClock.restart();
		}
		else;


		//Clear the window and draw everything again
        window.clear();

		window.draw(backgroundSprite);
		
        window.draw(leapRectangle);
		window.draw(leapSprite);
        window.draw(crewRectangle);
		window.draw(crewSprite);
        window.draw(gasGaugeRectangle);
		window.draw(gasGaugeSprite);
		//window.draw(gasText);
        window.draw(tireGaugeRectangle);
		window.draw(tireGaugeSprite);
		//window.draw(tireText);

		
		if(!robotArmActive && !loadingActive)
			//Set up the main menu
		{
			window.draw(gasRectangle);
			window.draw(gasSprite);
			window.draw(tireRectangle);
			window.draw(tireSprite);
			window.draw(helpRectangle);
			window.draw(helpSprite);
			window.draw(handRectangle);
			window.draw(handSprite);
		}
		else if(loadingActive)
		{
			window.draw(robotLoadingRectangle);
			window.draw(robotLoadingText);
		}
		else 
			//Set up the control menu
		{
			window.draw(robotUpLeftRectangle);
			window.draw(robotUpLeftSprite);
			window.draw(robotUpRectangle);
			window.draw(robotUpSprite);
			window.draw(robotUpRightRectangle);
			window.draw(robotUpRightSprite);
			window.draw(robotLeftRectangle);
			window.draw(robotLeftSprite);
			window.draw(robotStopRectangle);
			window.draw(robotStopSprite);
			window.draw(robotRightRectangle);
			window.draw(robotRightSprite);
			window.draw(robotDownLeftRectangle);
			window.draw(robotDownLeftSprite);
			window.draw(robotDownRectangle);
			window.draw(robotDownSprite);
			window.draw(robotDownRightRectangle);
			window.draw(robotDownRightSprite);
			window.draw(robotExitRectangle);
			window.draw(robotExitSprite);
		}

		if(resetActive)
		{
			window.draw(resetText);
			window.draw(resetConfirmationButtonRectangle);
			window.draw(resetCountdownText);
			window.draw(resetButtonText);
		}

		if(helpActive)
		{
			//Put all the text in here.
			window.draw(helpGasGaugeText);
			window.draw(helpTireGaugeText);
			window.draw(helpCrewText);
			if(!loadingActive && !robotArmActive){
				window.draw(helpGasText);
				window.draw(helpTireText);
				window.draw(helpResetText);
				window.draw(helpHelpText);
			}
		}

		if(initialLoad){
			window.draw(backgroundRectangle);
			window.draw(initializeText);
		}


        window.display();

		if(initialLoad && initialClock.getElapsedTime().asMilliseconds() > INITIALIZE_TIME)
			initialLoad = 0;
    }
	
	RS232_CloseComport(ROBOT_PORT_NUMBER);
	RS232_CloseComport(CAR_PORT_NUMBER);

	// Remove the sample listener when done
	controller.removeListener(listener);
  
    return 0;

}