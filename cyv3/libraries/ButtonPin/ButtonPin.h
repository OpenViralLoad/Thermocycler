/*


*/

#ifndef BUTTONPIN_H
#define BUTTONPIN_H


class ButtonPin {


	private:
		int BPIN_SELECT, BPIN_UP, BPIN_DOWN;
		int BPIN_LEFT, BPIN_RIGHT;
		int pin;

	public:
		ButtonPin( int pin_ );
		~ButtonPin();
		int readButtons();

};
#endif
