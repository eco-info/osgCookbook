// Primeiro terreno controlado por mouse e teclado
// derivado de https://github.com/SunilRao01/OSGFirstPersonController
// vpbmaster -v 0.005 -l 6 -d BRalt.tif -t br.png -o terreno1/BR.osgb

/* To-do:
 * 
 . Tecla Home: voltar pra posição inicial
 . Posição do ponto indicado pelo cursor
 . Impedir borda de trancar o cursor
 . Desenhar cruz central (só com !mouseFree)
 * Desenhar bússola (Heading) (removível)
 * Usar mapa com oceano azul
 * Calcular altitude real
 * Botão direito do mouse (?): navegar na mesma altitude
 / Impedir câmera de passar do chão
 * acrescentar luzes e objetos
 * mudar cálculos do handle para outro lugar?
 * mudar FirstPersonManipulator para StandardManipulator?
 * criar controlador a pé
 * Intro: aproximando velozmente do "infinito", desacelerando até a posição inicial sobre o país
 * 
 */

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <osgDB/ReadFile>
#include <osgGA/FirstPersonManipulator>
#include <osgViewer/Viewer>
#include <osgText/Text>

/* create an ordinary camera node which will be rendered on the top after
the main scene is drawn. It can be used to display some heads-up display (HUD) texts
and images. */
osg::Camera* createHUDCamera( double left, double right, double bottom, double top )
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
	camera->setClearMask( GL_DEPTH_BUFFER_BIT );
	camera->setRenderOrder( osg::Camera::POST_RENDER );
	camera->setAllowEventFocus( false );
	camera->setProjectionMatrix(
	osg::Matrix::ortho2D(left, right, bottom, top) );
	camera->getOrCreateStateSet()->setMode(
	GL_LIGHTING, osg::StateAttribute::OFF );
	return camera.release();
}

// function for creating HUD texts
//osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("Charter.ttf");
osgText::Text* createText( const osg::Vec3& pos, const std::string& content, float size )
{
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setDataVariance( osg::Object::DYNAMIC );
	//text->setFont( g_font.get() );
	text->setCharacterSize( size );
	text->setAxisAlignment( osgText::TextBase::XY_PLANE );
	text->setPosition( pos );
	text->setText( content );
	return text.release();
}

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
	virtual std::string pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
	virtual bool performMovement();
	virtual bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	osgViewer::Viewer *_viewer;
	osg::Timer mainTimer;
};

const float acc = 0.1;
const float maxVel = 1;
osg::Vec3d tempMov;
float velX = 0, velY = 0, velZ = 0;	// XYZ: velocidades lateral, vertical e frontal
float camAlt = 0, camLat = 0, camLon = 0, ptrAlt = 0, ptrLat = 0, ptrLon = 0;
bool keyW = false, keyA = false, keyS = false, keyD = false, keyR = false, keyF = false;
bool Homing = false, mouseFree = false, Warping = false;
const float inputTimeInterval = 0.02;
double maxTick = inputTimeInterval;
int windowX, windowY, windowW, windowH;
osgText::Text* text;
osgViewer::Viewer::Windows::iterator window;
osg::ref_ptr<osg::Geode> crossGeode;

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
	//double term3 = -2 * (qx*qz - qw*qy);
	double term4 = 2 * (qw*qx + qy*qz);
	double term5 = sqw - sqx - sqy + sqz;

	double heading = atan2(term1, term2);
	double pitch = atan2(term4, term5);
	//double roll = asin(term3);

	//Return values in degrees if requested, else its radians
	if (degrees)
	{
		heading = osg::RadiansToDegrees(heading);
		pitch   = osg::RadiansToDegrees(pitch);
		//roll    = osg::RadiansToDegrees(roll);
	}
	return osg::Vec3d(heading, pitch, 0); // (não usa roll)
}

bool OVNIController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	FirstPersonManipulator::handle(ea, aa); // Still use first person manipulator for camera movements (Inherited class function)
	if (!_viewer)
		return false;
	// Set the viewer's "eye" position, which is located at the center of the camera.
	osg::Vec3d eyePos;
	osg::Matrix matrix = _viewer->getCameraManipulator()->getMatrix();
	eyePos = matrix.getTrans();
	osg::Quat camRotation = matrix.getRotate();
	// Movement of the camera correlates with W, A, S, D
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
	{
		//std::cout << ea.getKey() << "\n";
		switch(ea.getKey())
		{
			case 65289: // Tab
				mouseFree = !mouseFree;
				(*window)->useCursor(mouseFree);
				if (mouseFree)
				{
					crossGeode->setNodeMask(0); // esconde a "mira"
					aa.requestWarpPointer( windowW/2, windowH/2 ); // põe o mouse no meio da tela
				}
				else
				{
					crossGeode->setNodeMask(1); // mostra a "mira"
				}
				break;
			case 65360: // Home
				Homing = true;
				break;
			case 'w':
				keyW = true;
				break;
			case 'a':
				keyA = true;
				break;
			case 's':
				keyS = true;
				break;
			case 'd':
				keyD = true;
				break;
			case 'r':
				keyR = true;
				break;
			case 'f':
				keyF = true;
				break;
		}
	}
	else
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
	{
		switch(ea.getKey())
		{
			case 'w':
				keyW = false;
				break;
			case 'a':
				keyA = false;
				break;
			case 's':
				keyS = false;
				break;
			case 'd':
				keyD = false;
				break;
			case 'r':
				keyR = false;
				break;
			case 'f':
				keyF = false;
				break;
		}
	}
	osg::Vec3 hpr = fromQuat(camRotation,true); // ângulos da câmera, em graus
	std::stringstream h, p, eyeX, eyeY, eyeZ;
	h << std::fixed << std::setprecision(1) << hpr.x(); // heading
	p << std::fixed << std::setprecision(1) << hpr.y(); // pitch

	camLat = eyePos.y();
	camLon = eyePos.x();
	if( eyePos.z() < 0.3 ) // altura mínima do OVNI
	{
		eyePos = osg::Vec3( camLon, camLat, 0.3 );
	}
	camAlt = eyePos.z();
	eyeY << std::fixed << std::setprecision(1) << camLat;
	eyeX << std::fixed << std::setprecision(1) << camLon;
	eyeZ << std::fixed << std::setprecision(2) << camAlt;

	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	std::string pickStr="";
	if (view)
	{
		if (mouseFree)
			pickStr = pick(view,ea); // coordenadas do mouse
		else
		{
			osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter(ea);
			event->setX(windowW/2);
			event->setY(windowH/2);
			pickStr = pick(view,*event); // coordenadas do centro da tela
		}
	}
	text->setText( "OVNI lat:lon:alt " + eyeY.str() + ":" + eyeX.str() + ":" + eyeZ.str() + "\nH/P " + h.str() + "/" + p.str() + "\n" + pickStr );

	eyePos += camRotation * tempMov * camAlt/10; // * camAlt/10 faz a velocidade aumentar com a altitude
	matrix.setTrans(eyePos);
	// Check [mainTimer.time % (time divisor) == 0]
	if (mainTimer.time_s() >= maxTick) // ??
	{
		_viewer->getCameraManipulator()->setByMatrix(matrix); // muda posição, não o ângulo
		maxTick += inputTimeInterval;
	}

	return false;
}

// examples/osgpick/osgpick.cpp:102
std::string OVNIController::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (view->computeIntersections(ea,intersections))
    {
		osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
		std::stringstream ptrX, ptrY, ptrZ;
		ptrLat = hitr->getWorldIntersectPoint().y();
		ptrLon = hitr->getWorldIntersectPoint().x();
		ptrAlt = hitr->getWorldIntersectPoint().z();
		ptrX << std::fixed << std::setprecision(1) << ptrLon;
		ptrY << std::fixed << std::setprecision(1) << ptrLat;
		ptrZ << std::fixed << std::setprecision(2) << ptrAlt;
		return "Cursor lat:lon:alt: " + ptrX.str() + ":" + ptrY.str() + ":" + ptrZ.str();
    }
	else
		return "";
}

bool OVNIController::performMovement()
{
	// return if less then two events have been added
	if( mouseFree || _ga_t0.get() == NULL || _ga_t1.get() == NULL )
	{
		if( Warping )
			mouseFree = false;
		return false;
	}
	if( Warping )
		Warping = false;
	// get delta time
	double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
	if( eventTimeDelta < 0. )
	{
		OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
		eventTimeDelta = 0.;
	}
	// get deltaX and deltaY
	float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
	float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();
	// return if there is no movement.
	if( dx == 0. && dy == 0. )
		return false;
	// call appropriate methods
	unsigned int buttonMask = _ga_t1->getButtonMask();
	unsigned int modKeyMask = _ga_t1->getModKeyMask();
	if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	else if( ( buttonMask == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON ) ||
			( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && modKeyMask & osgGA::GUIEventAdapter::MODKEY_CTRL ) ||
			( buttonMask == (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ) )
		return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
	else if( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
		return performMovementRightMouseButton( eventTimeDelta, dx, dy );
	else
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	return false;
}

bool OVNIController::handleMouseMove(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	addMouseEvent( ea );
	if ( performMovement() )
		aa.requestRedraw();
	aa.requestContinuousUpdate( false );
	_thrown = false;
	if( !mouseFree )
	{
		// Warp horizontal
		if( _ga_t0->getX() > windowW*0.9 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( windowW*0.2, _ga_t0->getY() );
		}
		else
		if( _ga_t0->getX() < windowW*0.1 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( windowW*0.8, _ga_t0->getY() );
		}
		// Warp vertical
		if( _ga_t0->getY() > windowH*0.9 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( _ga_t0->getX(), windowH*0.2 );
		}
		else
		if( _ga_t0->getY() < windowH*0.1 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( _ga_t0->getX(), windowH*0.8 );
		}
	}
	return true;
}

void calcAcc() // calcula aceleração do OVNI
{
	// velocidade frontal
	if (!keyW && !keyS)
		velZ *= 0.9f;
	else
	{
		if (keyW && velZ > -maxVel)
			velZ -= acc;
		if (keyS && velZ < maxVel)
			velZ += acc;
	}
	// velocidade lateral
	if (!keyA && !keyD)
		velX *= 0.9f;
	else
	{
		if (keyA && velX > -maxVel)
			velX -= acc;
		if (keyD && velX < maxVel)
			velX += acc;
	}
	// velocidade vertical
	if (!keyR && !keyF)
		velY *= 0.9f;
	else
	{
		if (keyR && velY < maxVel)
			velY += acc;
		if (keyF && velY > -maxVel)
			velY -= acc;
	}
	tempMov.set(velX, velY, velZ);
}

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( osgDB::readNodeFile("BR.osgb") );

	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	osg::ref_ptr<OVNIController> controller = new OVNIController(&viewer);
	viewer.setCameraManipulator(controller);
	//viewer.setUpViewInWindow(0, 0, 800, 600);
	viewer.realize();
	// Posição inicial da câmera
	osg::Quat quad0;
	controller->setTransformation(osg::Vec3(-54.4f,-14.2666667f,80.0f), quad0 ); // camLon (-74-34.8)/2 = -54.4, camLat (5.3333333-33.8666667)/2 = -14.2666667, camAlt = 80
	// Detecta coordenadas da janela
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	window = windows.begin();
	(*window)->useCursor(false);
	(*window)->getWindowRectangle(windowX, windowY, windowW, windowH);
	std::cout << "Tamanho da janela: " << windowW << "x" << windowH << "\n";

	// Linhas do HUD
    crossGeode = new osg::Geode();
	osg::Geometry* linesGeom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array(4);
	int crossP = 20;
	(*vertices)[0].set(windowW/2 - crossP, windowH/2, 0);
	(*vertices)[1].set(windowW/2 + crossP, windowH/2, 0);
	(*vertices)[2].set(windowW/2, windowH/2 - crossP, 0);
	(*vertices)[3].set(windowW/2, windowH/2 + crossP, 0);
	linesGeom->setVertexArray(vertices);
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
	linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
	linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,4));
	crossGeode->addDrawable(linesGeom);
	crossGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED); // With lighting off, geometry color ignores the viewing angle
	// Texto do HUD
	text = createText(osg::Vec3(10, windowH-30, 0), "", 20);
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	textGeode->addDrawable( text );
	osg::ref_ptr<osg::Camera> hudCamera = createHUDCamera(0, windowW, 0, windowH); // cria um HUD do tamanho da janela, não mais 800x600 (piorou o desempenho?)
	hudCamera->addChild( textGeode.get() );
	hudCamera->addChild( crossGeode.get() );
	root->addChild( hudCamera.get() );
	while ( !viewer.done() )
	{
		calcAcc();
		if (Homing) // volta a câmera à posição inicial ao apertar Home
		{
			controller->setTransformation(osg::Vec3(-54.4f,-14.2666667f,80.0f), quad0 );
			Homing = false;
		}
		viewer.frame();
	}
	return 0;
}

// g++ terreno1.cpp -losg -losgDB -losgGA -losgText -losgViewer -o terreno1
