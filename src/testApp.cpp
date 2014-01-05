

#include "testApp.h"
#include "simulation_main.h"

//--------------------------------------------------------------
void testApp::setup(){
	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	ofBackground( 255, 255, 255 ); //Set white background
	sp_simulationPoints = simulation_main();
	percentDisplay = 100;
}

//--------------------------------------------------------------
void testApp::update(){

}

//--------------------------------------------------------------
void testApp::draw(){
	//int width = ofGetWidth();
	//int height = ofGetHeight();
	int width = ofGetWindowWidth();
	int height = ofGetWindowHeight();
	//ofSetColor( 0, 0, 0 ); //Set black color
	
	char str[20];
	sprintf(str,"x=%f",sp_simulationPoints->x[(int)(20000/10*(percentDisplay/100))]);
	ofDrawBitmapString(str,10,200);

	int x;
	for(x=1; x<(int)20000/10*(percentDisplay/100); x++){
		ofSetColor( 0, 0, 0 ); //Set black color
		float x1 = width/2 + sp_simulationPoints->x[x-1]*6;
		float x2 = width/2 + sp_simulationPoints->x[x]*6;
		float y1 = height/2 - sp_simulationPoints->y[x-1]*6;
		float y2 = height/2 - sp_simulationPoints->y[x]*6;
		ofLine( x1, y1, x2, y2);
		
		

		ofSetColor( 0, 0, 255 ); //Set blue color
		x1 = width/2 + sp_simulationPoints->fusion_x[x-1]*6;
		x2 = width/2 + sp_simulationPoints->fusion_x[x]*6;
		y1 = height/2 - sp_simulationPoints->fusion_y[x-1]*6;
		y2 = height/2 - sp_simulationPoints->fusion_y[x]*6;
		ofLine( x1, y1, x2, y2);
	}

}	

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
	percentDisplay = x*100/((float)ofGetWindowWidth());
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
