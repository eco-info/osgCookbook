#include <osgDB/ReadFile>
#include <osgGA/FirstPersonManipulator>
#include <osgViewer/Viewer>
#include <iostream>

class OVNIController : public osgGA::FirstPersonManipulator
{
public:
	OVNIController(osgViewer::Viewer *inputViewer) : _viewer(inputViewer)
	{
		// Start frame timer
		mainTimer.setStartTick();
		setAllowThrow(false); // não continua movendo ao soltar o botão do mouse
	}
	// Override handle function for keyboard input
	virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &);
	virtual bool performMovement();
	virtual bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	osgViewer::Viewer *_viewer;
	osg::Timer mainTimer;
};

const float moveSpeed = 0.6;
osg::Vec3d tempMov;
const float inputTimeInterval = 0.02;
double maxTick = inputTimeInterval;
int windowX, windowY, windowW, windowH;

bool OVNIController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// Still use first person manipulator for camera movements (Inherited class function)
	FirstPersonManipulator::handle(ea, aa);
	if (!_viewer)
		return false;
	// Set the viewer's "eye" position, which is located at the center of the camera.
	osg::Vec3d eyePos;
	osg::Matrix matrix = _viewer->getCameraManipulator()->getMatrix();
	eyePos = matrix.getTrans();
	osg::Quat camRotation = matrix.getRotate();
	switch(ea.getEventType())
	{
		case(osgGA::GUIEventAdapter::KEYDOWN):
			// Movement of the camera correlates with W, A, S, D
			switch(ea.getKey())
			{
				case 'w':
					tempMov.z() = -moveSpeed;
					break;
				case 'a':
					tempMov.x() = -moveSpeed;
					break;
				case 's':
					tempMov.z() = moveSpeed;
					break;
				case 'd':
					tempMov.x() = moveSpeed;
			}
			break;
		case(osgGA::GUIEventAdapter::KEYUP):
		{
			switch(ea.getKey())
			{
				case 'w':
					tempMov.set(0, 0, 0);
					break;
				case 'a':
					tempMov.set(0, 0, 0);
					break;
				case 's':
					tempMov.set(0, 0, 0);
					break;
				case 'd':
					tempMov.set(0, 0, 0);
			}
		}
	}
	eyePos += camRotation * tempMov;
	matrix.setTrans(eyePos);
	// Check [mainTimer.time % (time divisor) == 0]
	if (mainTimer.time_s() >= maxTick)
	{
		_viewer->getCameraManipulator()->setByMatrix(matrix);
		maxTick += inputTimeInterval;
	}
	return false;
}

osg::Vec3d fromQuat(const osg::Quat& quat, bool degrees)
{
	// From: http://guardian.curtin.edu.au/cga/faq/angles.html
	// Except OSG exchanges pitch & roll from what is listed on that page
	double qx = quat.x();
	double qy = quat.y();
	double qz = quat.z();
	double qw = quat.w();

	double sqx = qx * qx;
	double sqy = qy * qy;
	double sqz = qz * qz;
	double sqw = qw * qw;

	double term1 = 2 * (qx*qy + qw*qz);
	double term2 = sqw + sqx - sqy - sqz;
	double term3 = -2 * (qx*qz - qw*qy);
	double term4 = 2 * (qw*qx + qy*qz);
	double term5 = sqw - sqx - sqy + sqz;

	double heading = atan2(term1, term2);
	double pitch = atan2(term4, term5);
	double roll = asin(term3);

	//Return values in degrees if requested, else its radians
	if (degrees)
	{
		heading = osg::RadiansToDegrees(heading);
		pitch   = osg::RadiansToDegrees(pitch);
		roll    = osg::RadiansToDegrees(roll);
	}

	return osg::Vec3d(heading, pitch, roll);
}

bool OVNIController::performMovement()
{
	// return if less then two events have been added
	if( _ga_t0.get() == NULL || _ga_t1.get() == NULL )
		return false;

	// get delta time
	double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
	if( eventTimeDelta < 0. )
	{
		OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
		eventTimeDelta = 0.;
	}

	// get deltaX and deltaY
	/*if (_ga_t0->getX() == windowW/2 && _ga_t0->getY() == windowH/2)
		return false;*/

	//std::cout << _ga_t0->getX() << "|" << _ga_t0->getX() << "\n";
	float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
	float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();

	// return if there is no movement.
	if( dx == 0. && dy == 0. )
		return false;

	// call appropriate methods
	unsigned int buttonMask = _ga_t1->getButtonMask();
	unsigned int modKeyMask = _ga_t1->getModKeyMask();
	if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
	{
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	}
	else if( ( buttonMask == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON ) ||
		( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && modKeyMask & osgGA::GUIEventAdapter::MODKEY_CTRL ) ||
		( buttonMask == (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ) )
	{
		return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
	}
	else if( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
	{
		return performMovementRightMouseButton( eventTimeDelta, dx, dy );
	}
	else
	{
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	}

	return false;
}

bool OVNIController::handleMouseMove(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	addMouseEvent( ea );

	if ( performMovement() )
	{
		aa.requestRedraw();

		/*osg::Matrix matrix;
		osg::Vec3 hpr;
		double h, p, r;
		matrix = _viewer->getCamera()->getViewMatrix();
		hpr = fromQuat(matrix.getRotate(),true); // ângulos da câmera, em graus
		h = hpr.x();
		p = hpr.y();
		r = hpr.z();
		std::cout << h << "|" << p << "|" << r << "\n";*/
	}

	aa.requestContinuousUpdate( false );
	_thrown = false;

	//centerMousePointer( ea, aa );
	return true;
}

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( osgDB::readNodeFile("BR.osgb") );
	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	osg::ref_ptr<OVNIController> controller = new OVNIController(&viewer);
	//osg::ref_ptr<osgGA::UFOManipulator> controller = new osgGA::UFOManipulator();
	viewer.setCameraManipulator(controller);
	//viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
	//viewer.setUpViewInWindow(0, 0, 800, 600);
	viewer.realize();
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	for(auto &window : windows) {
		//window->useCursor(false);
		//window->setCursor(osgViewer::GraphicsWindow::NoCursor);
		window->getWindowRectangle(windowX, windowY, windowW, windowH);
		std::cout << "Tamanho da janela: " << windowW << "x" << windowH << "\n";
	}
	while ( !viewer.done() )
	{
		viewer.frame();
	}
	return 0;
	//return viewer.run();
}

// g++ terrain1.cpp -losg -losgDB -losgGA -losgViewer -o terrain1
