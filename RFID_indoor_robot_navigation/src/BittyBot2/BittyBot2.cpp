#include "Arduino.h"
#include "BittyBot2.h"

BittyBot::BittyBot(int enableL, int enableR, int Left1, int Left2, int Right1, int Right2) 
{
  _enableL = enableL;
  _enableR = enableR;
  _Left1 = Left1;
  _Left2 = Left2;
  _Right1 = Right1;
  _Right2 = Right2;
  _previousMillis = 0;
}

void BittyBot::begin() {
  
	pinMode(_enableL, OUTPUT);
	pinMode(_enableR, OUTPUT);
	pinMode(_Left1, OUTPUT);
	pinMode(_Left2, OUTPUT);
	pinMode(_Right1, OUTPUT);
	pinMode(_Right2, OUTPUT);
	//calibrate();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, LOW);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, LOW);
	BittyBot::Speed(0,0);

}
 
void BittyBot::Speed(int speedL, int speedR) {
	_speedL = speedL;
	_speedR = speedR;
	//digitalWrite(_Left1, LOW);
	//digitalWrite(_Left2, LOW);
	//digitalWrite(_Right1, LOW);
	//digitalWrite(_Right2, LOW);
	analogWrite(_enableL, _speedL);
	analogWrite(_enableR, _speedR);

}

void BittyBot::stop() {
    _previousMillis = millis();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, LOW);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, LOW);
	BittyBot::Speed(0,0);
	_isrunning = 0;
}

void BittyBot::forward(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, HIGH);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, HIGH);
	_OnTime = OnTime;
	_isrunning = 1;
}

void BittyBot::back(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, HIGH);
	digitalWrite(_Left2, LOW);
	digitalWrite(_Right1, HIGH);
	digitalWrite(_Right2, LOW);
	_OnTime = OnTime;
	_isrunning = 1;

}

void BittyBot::rightTight(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, HIGH);
	digitalWrite(_Right1, HIGH);
	digitalWrite(_Right2, LOW);
	_OnTime = OnTime;
	_isrunning = 1;
}

void BittyBot::leftTight(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, HIGH);
	digitalWrite(_Left2, LOW);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, HIGH);
	_OnTime = OnTime;
	_isrunning = 1;
}

void BittyBot::right(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, HIGH);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, LOW);
	analogWrite(_enableR, 0);
	_OnTime = OnTime;
	_isrunning = 1;
}

void BittyBot::left(int OnTime) {
    _previousMillis = millis();
	digitalWrite(_Left1, LOW);
	digitalWrite(_Left2, LOW);
	digitalWrite(_Right1, LOW);
	digitalWrite(_Right2, HIGH);
	analogWrite(_enableL, 0);
	_OnTime = OnTime;
	_isrunning = 1;
}

void BittyBot::update() {
	unsigned long currentMillis = millis();
	if (currentMillis - _previousMillis >= _OnTime) {
		_previousMillis = currentMillis;
		stop();
		BittyBot::Speed(_speedL, _speedR);
	}
}

void BittyBot::calibrate() {
//We need to know where the magnets are at the start, that is what this is suppose to do, give a common starting point.
	BittyBot::Speed(50,50);
		while (digitalRead(3) != HIGH) {
		
		right(1000);
			}
			stop();
			BittyBot::Speed(50,50);
		while (digitalRead(2) != HIGH) {
		
		left(1000);
	}
	stop();
	BittyBot::Speed(0,0);
}

int BittyBot::IsRunning() {
	return (_isrunning);

};
