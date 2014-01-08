#include <iostream>
#include <sstream>
#include <string>

using namespace std;

char tankGame(char tank);
char tireGame(char tire);

int main(){

	char flags = 0;

	char tank = 57;
	char tire = 57;


	string input = "go";

	while( input.compare("quit") ) {

		cout << "tank: " << tank << '\n';
		cout << "tire: " << tire << '\n' << '\n';

		if( flags == 0 ){
			cout << "state: ";
			getline(cin,input);
			cout << '\n';
		
			if( input.compare("tank")==0 ) {
				flags |= 0x01;
			}
			else if( input.compare("tire") == 0) {
				flags |= 0x02;
			}
			else {
				if(tank > 48){
					tank = tank - 1;
				}
				if(tire > 48){
					tire = tire -1;
				}
			}
		}

		else if( (flags & 0x01) == 0x01 ){
			tank = tankGame(tank);
			if(tank == 57){
				cout << "filled" << '\n' << '\n';
				flags &= 0xFE;
			}
		}
		else if( (flags & 0x02) == 0x02 ){
			tire = tireGame(tire);
			if(tire == 57){
				cout << "fixed" << '\n' << '\n';
				flags &= 0xFD;
			}
		}
		else {
			//do nothing
		}
				
		
	}
	cout << "finished";
	return 0;
}

char tankGame(char tank){
	if(tank == 57)
		return tank;
	string input;
	int value = 0;
	getline(cin, input);
	if( input.empty() ){
		value = 1000; //so I can just enter through the fill;
	}
	else if( input.at(0)=='c' ){
		value = input.at(1) << 4;
		value += input.at(2);
	}
	else {
		//do nothing
	}
	if( value > 600 ) {
		tank++;
	}
	else {
		//tire = tire
	}

	return tank;
}


char tireGame(char tire){
	if(tire == 57)
		return tire;
	string input;
	int value = 30;
	getline(cin, input);
	if( input.empty() ){
		value = 0; //so I can just enter through the fill;
	}
	else if( input.at(0)=='r' ){
		value = input.at(1) << 4;
		value += input.at(2);

	}
	else {
		//do nothing
	}
	if( value < 20 ) {
		tire++;
	}
	else {
		//tire = tire
	}

	return tire;
}