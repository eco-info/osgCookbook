// Primeiro terreno controlado por mouse e teclado
// derivado de https://github.com/SunilRao01/OSGFirstPersonController
// vpbmaster -v 0.005 -l 6 -d BRalt.tif -t br.png -o terreno1/BR.osgb

/* To-do:
 * 
 . Tecla Home: voltar pra posição inicial
 . Posição do ponto indicado pelo cursor
 . Impedir borda de trancar o cursor
 . Desenhar cruz central (só com !mouseFree)
 . Impedir câmera de passar do chão
 . Desenhar bússola (Heading) (removível)
 . Desenhar retângulos translúcidos no HUD
 * Barra de espaço jogando pra posição inicial anterior
 * Fazer a velocidade do movimento não depender dos FPS
 * Testar em outro PC, sem OSG instalado
 * Trocar espaços no início das linhas por tabs
 * Criar botões clicáveis no HUD (ligar e desligar bússola, linhas das UFs, etc)
 * Bússola e outros elementos devem escalar com o tamanho da janela? Quanto?
 * Usar mapa com oceano azul
 * Calcular altitude real
 * Botão direito do mouse (?): navegar na mesma altitude
 * Botão direito do mouse travando movimento?
 * acrescentar luzes e objetos
 / desenhar shapefiles
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
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgGA/FirstPersonManipulator>
#include <osgText/Text>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>
#include <sys/stat.h>

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
	virtual float pickDown( osg::Vec3d pos );
	virtual bool performMovement();
	virtual bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	osgViewer::Viewer *_viewer;
	osg::Timer mainTimer;
};

osg::ref_ptr<OVNIController> controller;
//osg::ref_ptr<osg::Group> shpUFs;
const float acc = 0.1;
const float maxVel = 1;
osg::Vec3d tempMov;
float velX = 0, velY = 0, velZ = 0;	// XYZ: velocidades lateral, vertical e frontal
float camAlt = 0, camLat = 0, camLon = 0, ptrAlt = 0, ptrLat = 0, ptrLon = 0;
bool keyW = false, keyA = false, keyS = false, keyD = false, keyR = false, keyF = false;
bool Homing = false, mouseFree = false, Warping = false, mostraBussola = true;
const float inputTimeInterval = 0.02;
double maxTick = inputTimeInterval;
int windowX, windowY, windowW, windowH;
osgText::Text* text;
osgViewer::Viewer::Windows::iterator window;
osg::ref_ptr<osg::Geode> crossGeode;
osg::ref_ptr<osg::PositionAttitudeTransform> bussola;
osg::ref_ptr<osg::PositionAttitudeTransform> menu;

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
					menu->setNodeMask(1); // mostra o menu
					aa.requestWarpPointer( windowW/2, windowH/2 ); // põe o mouse no meio da tela
				}
				else
				{
					crossGeode->setNodeMask(1); // mostra a "mira"
					menu->setNodeMask(0); // esconde o menu
				}
				break;
			case 65360: // Home
				Homing = true;
				break;
			case 'b': // Bússola
				mostraBussola = !mostraBussola;
				if (mostraBussola)
				{
					bussola->setNodeMask(1); // mostra a bússola
				}
				else
				{
					bussola->setNodeMask(0); // esconde a bússola
				}
				break;
			/*case 'u': // mostra/esconde as UFs
				shpUFs->setNodeMask(!shpUFs->getNodeMask()); // e se algum outro bit for usado, corremos esse risco (talvez no futuro) ??
				break;*/
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
	std::stringstream /*h, p, */eyeX, eyeY, eyeZ;
	//h << std::fixed << std::setprecision(1) << hpr.x(); // heading
	//p << std::fixed << std::setprecision(1) << hpr.y(); // pitch

	camLat = eyePos.y();
	camLon = eyePos.x();
	float chao = pickDown( eyePos );
	if( chao > 0 && eyePos.z() < chao+0.2 ) // altura mínima do OVNI
	{
		eyePos = osg::Vec3( camLon, camLat, chao+0.2 );
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
	bussola->setAttitude(osg::Quat(osg::inDegrees(-hpr.x()),osg::Vec3d(0,0,1)));
	//text->setText( "OVNI lat:lon:alt " + eyeY.str() + ":" + eyeX.str() + ":" + eyeZ.str() + "\nH/P " + h.str() + "/" + p.str() + "\n" + pickStr );
	text->setText( "OVNI lat:lon:alt " + eyeY.str() + ":" + eyeX.str() + ":" + eyeZ.str() + "\n" + pickStr );

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

// https://groups.google.com/forum/#!searchin/osg-users/LineSegmentIntersector|sort:date/osg-users/f4WZnzr8X5w/37pAZ4QqAwAJ
// src/osgUtil/LineSegmentIntersector.cpp
float OVNIController::pickDown( osg::Vec3d pos )
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	osg::Vec3d pos1( pos.x(), pos.y(), pos.z()+100 );
	osg::Vec3d pos2( pos.x(), pos.y(), -100 );
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intsec = new osgUtil::LineSegmentIntersector( osgUtil::Intersector::MODEL, pos1, pos2 );
	osgUtil::IntersectionVisitor iv( intsec.get() );
	_viewer->getSceneData()->accept( iv );
	if( intsec->containsIntersections() )
	{
		osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intsec->getIntersections().begin();
		return hitr->getWorldIntersectPoint().z();
	}
	else
		return -1;
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

void criaMenus() // examples/osggeometry/osggeometry.cpp
{
    /*osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile("compass.png");
    if (!image)
		return;*/
    osg::Geometry* polyGeom = new osg::Geometry();
    int mnuW = windowW;
    int mnuH = 100;
    osg::Vec3 myCoords[] =
    {
        osg::Vec3(0,0,0),
        osg::Vec3(0,-mnuH,0),
        osg::Vec3(mnuW,-mnuH,0),
        osg::Vec3(mnuW,0,0)
    };
    polyGeom->setVertexArray(new osg::Vec3Array(4,myCoords));
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0,0,0,0.5)); // alpha 0.5 = translúcido
    polyGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    polyGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
    osg::Vec2 myTexCoords[] =
    {
        osg::Vec2(0,1),
        osg::Vec2(0,0),
        osg::Vec2(1,0),
        osg::Vec2(1,1)
    };
    polyGeom->setTexCoordArray(0,new osg::Vec2Array(4,myTexCoords));
    unsigned short myIndices[] = { 0, 1, 3, 2 };
    polyGeom->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLE_STRIP,4,myIndices));
    osg::StateSet* stateset = new osg::StateSet;
    /*osg::Texture2D* texture = new osg::Texture2D;
    texture->setImage(image);
    stateset->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);*/
    stateset->setMode(GL_BLEND, osg::StateAttribute::ON); // deixa fundo do PNG transparente
    polyGeom->setStateSet(stateset);
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(polyGeom);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    menu = new osg::PositionAttitudeTransform();
	menu->setNodeMask(0); // menu começa oculto
	menu->setPosition(osg::Vec3d(0,windowH,-1));
    menu->addChild(geode);
}

void criaBussola() // examples/osggeometry/osggeometry.cpp
{
    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile("compass.png");
    if (!image)
		return;
    osg::Geometry* polyGeom = new osg::Geometry();
    int bussP = 64; // porque compass.png é 128x128 pixels
    osg::Vec3 myCoords[] =
    {
        osg::Vec3(-bussP,bussP,0),
        osg::Vec3(-bussP,-bussP,0),
        osg::Vec3(bussP,-bussP,0),
        osg::Vec3(bussP,bussP,0)
    };
    polyGeom->setVertexArray(new osg::Vec3Array(4,myCoords));
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); // 0.5 no final deixa translúcido
    polyGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    polyGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
    osg::Vec2 myTexCoords[] =
    {
        osg::Vec2(0,1),
        osg::Vec2(0,0),
        osg::Vec2(1,0),
        osg::Vec2(1,1)
    };
    polyGeom->setTexCoordArray(0,new osg::Vec2Array(4,myTexCoords));
    unsigned short myIndices[] = { 0, 1, 3, 2 };
    polyGeom->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLE_STRIP,4,myIndices));
    osg::StateSet* stateset = new osg::StateSet;
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setImage(image);
    stateset->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);
    stateset->setMode(GL_BLEND, osg::StateAttribute::ON); // deixa fundo do PNG transparente
    polyGeom->setStateSet(stateset);
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(polyGeom);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    bussola = new osg::PositionAttitudeTransform();
	bussola->setPosition(osg::Vec3d(windowW-bussP,bussP,0));
    bussola->addChild(geode);
}
/*
uint32_t leBig(std::ifstream& f)
{
    char *buf;
    uint32_t num;
    uint8_t num1, num2, num3, num4;
    buf = new char[4];
    f.read(buf,4);
    num1 = buf[0];
    num2 = buf[1];
    num3 = buf[2];
    num4 = buf[3];
    num = num1*0x1000000 +
          num2*0x10000   +
          num3*0x100     +
          num4;
    delete[] buf;
    return num;
}

uint32_t leLittle(std::ifstream& f)
{
    char *buf;
    uint32_t num;
    uint8_t num1, num2, num3, num4;
    buf = new char[4];
    f.read(buf,4);
    num1 = buf[0];
    num2 = buf[1];
    num3 = buf[2];
    num4 = buf[3];
    num = num4*0x1000000 +
          num3*0x10000   +
          num2*0x100     +
          num1;
    delete[] buf;
    return num;
}

double readDouble(std::ifstream& f)
{
    double X;
    f.read(reinterpret_cast<char*>(&X),8);
    return X;
}

//void linhaTerreno(Ogre::ManualObject* linha1,Ogre::Real x,Ogre::Real z)
//{
//    linha1->position(x,mTerrainGroup->getHeightAtWorldPosition(x,0,z),z);
//}

bool FileExists(std::string strFilename)
{
	struct stat stFileInfo;
	return (stat(strFilename.c_str(),&stFileInfo) == 0);
}

float recSizeT = 0;

//void readLevel4Rec(std::ifstream& f,Ogre::ManualObject* linha)
void readLevel4Rec(std::ifstream& f)
{
    char *buf;
    uint32_t recNo, recLen, tipo,
        nParts, nPoints, i, part;
    double X, Y;
    size_t j;
    std::vector<ulong> parts;
    recNo = leBig(f);
    recLen = leBig(f);
    tipo = leLittle(f);
    int recSize = 0;
    float altXY;
    // File format explained in "ESRI Shapefile Technical Description" (shapefile.pdf) - www.esri.com
    if (tipo == 3 || tipo == 5) // PolyLine or Polygon
    {
        buf = new char[32];
        f.read(buf,32); // reads the bounding box for that record (I won't use it)
        delete[] buf;         // (so I delete it)
        // reads number of parts and points of this record/polygon
        nParts = leLittle(f);
        nPoints = leLittle(f);
        // read all the parts at once and put their positions (relative to this record) in parts' vector
        // first part always begins at 0
        for ( i=0 ; i<nParts ; i++ )
        {
            part = leLittle(f);
            parts.push_back(part);
        }
        if (nParts > 1)
            part = parts[1]; // where the first part will end (the beginning of the 2nd part [1])
        else
            part = 0;
		// Linhas das UFs sobre o mapa
		osg::ref_ptr<osg::Geode> shpGeode = new osg::Geode();
		osg::Geometry* linesGeom = new osg::Geometry();
		osg::Vec3Array* vertices = new osg::Vec3Array;
        j = 1; // reading first part
        for ( i=0 ; i<nPoints ; i++ )
        {
            X = readDouble(f); // longitude
            Y = readDouble(f); // latitude
			//std::cout << "	X|Y = " << X << "|" << Y << "\n";
            //X = newSide*pixelSize*(X-LONGMIN)/(LONGMAX-LONGMIN) - newSide*pixelSize/2; // map X
            //Y = newSide*pixelSize*(LATMAX-Y)/(LATMAX-LATMIN) - newSide*pixelSize/2;    // map Y
            if (i == part) // end of the current part (j) or first time of 1 part's record
            {
                if (i > 0) // won't do if record has only 1 part (and i=part=0)
                {
                    //linha->end(); // jump to another polygon
					linesGeom->setVertexArray(vertices);
					osg::Vec4Array* colors = new osg::Vec4Array;
					colors->push_back(osg::Vec4(0,0,0,1));
					linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
					osg::Vec3Array* normals = new osg::Vec3Array;
					normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
					linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
					linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,recSize));
					shpGeode->addDrawable(linesGeom);
					shpGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED); // With lighting off, geometry color ignores the viewing angle
					shpUFs->addChild( shpGeode.get() );
					recSize = 0;
                    //linha->begin();
					shpGeode = new osg::Geode();
					linesGeom = new osg::Geometry();
					vertices = new osg::Vec3Array;
                }
                //altXY = controller->pickDown( osg::Vec3d(X,Y,1) );
				//vertices->push_back(osg::Vec3d(X,Y,altXY));
				vertices->push_back(osg::Vec3d(X,Y,1));
				recSize++;
				recSizeT++;
                //linhaTerreno(linha,X,Y); // add point to current polygon
                if (j < parts.size()) // adjust limit (end point) of next part
                {
                    j++;
                    if (j < parts.size())
                        part = parts[j];
                    else
                        part = -1; // will end the loop by i=nPoints (since its the last part)
                }
            }
            else // i != part, all points inside any part
            {
                //altXY = controller->pickDown( osg::Vec3d(X,Y,1) );
				//vertices->push_back(osg::Vec3d(X,Y,altXY));
				vertices->push_back(osg::Vec3d(X,Y,1));
				recSize++;
				recSizeT++;
                //linhaTerreno(linha,X,Y);
            }
        } // for i
        // draw remaining points
        //linha->end();
		//int numCoords = sizeof(vertices)/sizeof(osg::Vec3);
		linesGeom->setVertexArray(vertices);
		osg::Vec4Array* colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(0,0,0,1));
		linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
		osg::Vec3Array* normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
		linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
		linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,recSize));
		shpGeode->addDrawable(linesGeom);
		shpGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED); // With lighting off, geometry color ignores the viewing angle
		shpUFs->addChild( shpGeode.get() );
    } // if (tipo==3 || tipo==5)
}

//void drawLevel4(Ogre::ManualObject* linha)
bool drawLevel4()
{
	bool funcionou = false;
    char *offsetHeader;
    ulong fsize;
    std::string fname = "shp/br.shp";
    if (FileExists(fname))
    {
        std::ifstream fLevel4;
        fLevel4.open(fname.c_str(),std::ios::binary);
        if (fLevel4.is_open())
        {
			shpUFs = new osg::Group;
            offsetHeader = new char[72];
            fLevel4.read(offsetHeader,24); // reads the begin of the file's header
            fsize = leBig(fLevel4)*2;
            fLevel4.read(offsetHeader,72); // reads the remaining of the file's header
            delete[] offsetHeader;
            while (fLevel4.tellg() < fsize)
            {
				//std::cout << "fLevel4.tellg = " << fLevel4.tellg() << "\n";
                //linha->begin("matLinha",Ogre::RenderOperation::OT_LINE_STRIP);
                //readLevel4Rec(fLevel4,linha); // read and draw the roads
                readLevel4Rec(fLevel4); // read and draw the roads
            }
            funcionou = true;
            fLevel4.close();
            std::cout << "Total de pontos lidos (UFs) = " << recSizeT << "\n"; // 19481, leva mais ou menos 2 segundos pra calcular altitude de todos
        }
        else std::cout << "Problema ao abrir arquivo " << fname << '\n';
    }
    return funcionou;
}
*/

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( osgDB::readNodeFile("BR.osgb") );

	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	//osg::ref_ptr<OVNIController> controller = new OVNIController(&viewer);
	controller = new OVNIController(&viewer);
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

	// Linhas 3D
	/*if ( drawLevel4() )
		root->addChild( shpUFs.get() );*/

	// Linhas do HUD (examples/osggeometry/osggeometry.cpp)
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
	text = createText(osg::Vec3(10, windowH-30, 0), "", 25);
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	textGeode->addDrawable( text );
	osg::ref_ptr<osg::Camera> hudCamera = createHUDCamera(0, windowW, 0, windowH); // cria um HUD do tamanho da janela, não mais 800x600 (piorou o desempenho?)
	hudCamera->addChild( crossGeode.get() );
    criaBussola();
    criaMenus();
    hudCamera->addChild( bussola );
    hudCamera->addChild( menu );
	hudCamera->addChild( textGeode.get() );
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

// g++ terreno1.cpp -losg -losgDB -losgGA -losgText -losgUtil -losgViewer -o terreno1
