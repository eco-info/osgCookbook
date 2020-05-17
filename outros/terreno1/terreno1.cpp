// Primeiro terreno controlado por mouse e teclado
// derivado de https://github.com/SunilRao01/OSGFirstPersonController
// vpbmaster -v 0.005 -l 6 -d BRalt.tif -t br.png -o terreno1/BR.osgb
// Lista de classes: https://codedocs.xyz/openscenegraph/OpenSceneGraph/annotated.html

/* To-do:
 * 
 . Tecla Home: voltar pra posição inicial
 . Posição do ponto indicado pelo cursor
 . Impedir borda de trancar o cursor
 . Desenhar cruz central (só com !mouseFree)
 . Impedir câmera de passar do chão
 . Desenhar bússola (Heading) (removível)
 . Desenhar retângulos translúcidos no HUD
 . Barra de espaço jogando pra posição inicial anterior
 - Trocar espaços no início das linhas por tabs (procurar "  " | Geany menu Edit|Preferences|Editor|Display|Show white space)
 - acrescentar luzes (luz não ficou direcional...)
 - Inserir objetos 3d (podem representar grandes obras, problemas a resolver, etc) (definir a escala)
 - desenhar shapefiles (ok, mas não serve pra UFs)
 . Identificar os objetos 3d com o mouse
 . Mover os objetos 3d
 * Mostrar nomes das localidades
 * Dar nome aos objetos 3d dentro do jogo (osgWidget::Input?)
 * UFs com decalque, overlay, shaders, mudança da textura... ?
 * Mudar o tamanho da janela dentro do jogo
 * Estudar animações (para os objetos 3d e para a intro)
 * Intro: aproximando velozmente do "infinito", desacelerando até a posição inicial sobre o país (osglight.cpp:130)
 * Fazer a velocidade do movimento não depender dos FPS
 * Testar em outro PC, sem OSG instalado
 * Criar botões clicáveis no HUD (ligar e desligar bússola, linhas das UFs, etc)
 * Bússola e outros elementos devem escalar com o tamanho da janela? Quanto?
 * Usar mapa com oceano azul
 * Calcular altitude real
 * Botão direito do mouse (?): navegar na mesma altitude
 * Botão direito do mouse travando movimento?
 * mudar cálculos do handle para outro lugar?
 * mudar FirstPersonManipulator para StandardManipulator?
 * criar controlador a pé
 * 
 */

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/ValueObject>
#include <osgDB/ReadFile>
#include <osgFX/Outline>
#include <osgGA/FirstPersonManipulator>
//#include <osgSim/OverlayNode>
#include <osgText/Text>
#include <osgUtil/LineSegmentIntersector>
//#include <osgUtil/PrintVisitor>
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
	camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top) );
	camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
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
	virtual bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
	virtual bool handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
	virtual bool performMovement();
	virtual bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	osgViewer::Viewer *_viewer;
	osg::Timer mainTimer;
};

//osg::PI = 3.14159265358979323846;
const float pi = 3.141592653;
float lonC = (-74-34.8)/2; // -54.4
float latC = (5.3333333-33.8666667)/2; // -14.2666667
float rCeu = (-34.8+74)*2; // 78.4 = "raio do céu" (para calcular a posição inicial da câmera e a posição do "Sol")
float solLO=pi/4, solNS=pi/8; // ângulo do Sol, leste-oeste e norte-sul
osg::Vec3d posSol = osg::Vec3d( lonC+rCeu*sin(solLO), latC+rCeu*sin(solNS), rCeu*cos(solLO)*cos(solNS) );
osg::ref_ptr<OVNIController> controller;
const float acc = 0.1;
const float maxVel = 1;
osg::Vec3d camSpeed;
float velX=0, velY=0, velZ=0, velSol=0.01;	// XYZ: velocidades lateral, vertical e frontal
float camAlt = 0, camLat = 0, camLon = 0, ptrAlt = 0, ptrLat = 0, ptrLon = 0;
bool keyW=false, keyA=false, keyS=false, keyD=false, keyR=false, keyF=false, keyL=false, keyO=false, keyK=false, keyI=false;
bool Homing=false, mouseFree=false, Warping=false, mostraBussola=true, movendoBox=false;
const float inputTimeInterval = 0.02;
double maxTick = inputTimeInterval;
int windowX, windowY, windowW, windowH;
osgText::Text* text;
osgViewer::Viewer::Windows::iterator window;
osg::ref_ptr<osg::Node> Sol;
osg::ref_ptr<osg::Geode> crossGeode;
osg::ref_ptr<osg::PositionAttitudeTransform> bussola;
osg::ref_ptr<osg::PositionAttitudeTransform> menu;
osg::ref_ptr<osg::Group> blocos;
osg::Geode* selecionado = nullptr;
//osgUtil::PrintVisitor pv( std::cout );

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
		pitch = osg::RadiansToDegrees(pitch);
		//roll = osg::RadiansToDegrees(roll);
	}
	return osg::Vec3d(heading, pitch, 0); // (não usa roll)
}

void criaBloco( osg::Quat camRotation ) // ver examples/osgscribe/osgscribe.cpp para exemplos de linhas sobre superfícies
{
	osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable( new osg::Box );
	shape->setDataVariance( osg::Object::DYNAMIC );
	shape->setColor( osg::Vec4(1,0,0,1) ); // vermelho
	//shape->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON ); // deixa translúcido
	shape->setUseDisplayList( false );
	shape->setName( "Box " + std::to_string( blocos->getNumChildren() ) );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Vec3 hpr = fromQuat(camRotation,false); // ângulos da câmera, em radianos
	// hpr.x() = heading , hpr.y = pitch
	geode->setUserValue( "speed", osg::Vec3d(-0.1*sin(hpr.x())*sin(hpr.y()),0.1*cos(hpr.x())*sin(hpr.y()),-0.005) ); // velocidade "pra frente" (câmera) e pra baixo (gravidade)
	geode->setName( "GeodeBox " + std::to_string( blocos->getNumChildren() ) );
	geode->addChild( shape );

	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	trans->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) *
						osg::Matrix::translate( osg::Vec3(camLon, camLat, camAlt)+camRotation*osg::Vec3(0,0,-1) ) ); // posição: -1 = distância à frente da câmera
	trans->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
	trans->getOrCreateStateSet()->setAttributeAndModes( new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
	trans->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	trans->setName("MatrTransfBox " + std::to_string( blocos->getNumChildren() ) );
	trans->addChild( geode );

	blocos->addChild( trans );
	//blocos->accept( pv );
}

bool OVNIController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
		return handleMousePush( ea, aa );
	else
	if( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
		return handleMouseRelease( ea, aa );
	else
	if( ea.getEventType() == osgGA::GUIEventAdapter::MOVE ||
		ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
		return handleMouseMove( ea, aa ); // barra de espaços parou de levar pra coordenada inicial anterior
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
			case ' ': // Barra de espaço
				criaBloco( camRotation );
				break;
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
			case 'l':
				keyL = true;
				break;
			case 'o':
				keyO = true;
				break;
			case 'k':
				keyK = true;
				break;
			case 'i':
				keyI = true;
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
			case 'l':
				keyL = false;
				break;
			case 'o':
				keyO = false;
				break;
			case 'k':
				keyK = false;
				break;
			case 'i':
				keyI = false;
				break;
		}
	}
	osg::Vec3 hpr = fromQuat(camRotation,true); // ângulos da câmera, em graus
	std::stringstream eyeX, eyeY, eyeZ;

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
	text->setText( "OVNI lat:lon:alt " + eyeY.str() + ":" + eyeX.str() + ":" + eyeZ.str() + "\n" + pickStr );

	eyePos += camRotation * camSpeed * camAlt/10; // * camAlt/10 faz a velocidade aumentar com a altitude
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
		std::stringstream ptrX, ptrY, ptrZ;
		if( movendoBox && selecionado ) // arrastando algum Box (não precisa do "selecionado", mas por segurança...)
		{
			for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
				hitr != intersections.end();
				++hitr)
			{
				if( strcmp( hitr->drawable->className(),"Geometry" ) == 0 ) // terreno
				{
					osg::MatrixTransform* mtxTr = dynamic_cast<osg::MatrixTransform*>(selecionado->asNode()->getParent(0));
					if( mtxTr )
					{
						osg::Vec3d pos = hitr->getWorldIntersectPoint();
						ptrX << std::fixed << std::setprecision(1) << pos.x();
						ptrY << std::fixed << std::setprecision(1) << pos.y();
						ptrZ << std::fixed << std::setprecision(2) << pos.z();
						pos.z() += 0.05;
						mtxTr->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
						return "Movendo para lat:lon:alt: " + ptrX.str() + ":" + ptrY.str() + ":" + ptrZ.str();
					}
				}
			}
			return "";
		}
		else // não está arrastando nenhum Box
		{
			osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
			ptrLat = hitr->getWorldIntersectPoint().y();
			ptrLon = hitr->getWorldIntersectPoint().x();
			ptrAlt = hitr->getWorldIntersectPoint().z();
			ptrX << std::fixed << std::setprecision(1) << ptrLon;
			ptrY << std::fixed << std::setprecision(1) << ptrLat;
			ptrZ << std::fixed << std::setprecision(2) << ptrAlt;
			osg::NodePath path = hitr->nodePath;
			bool achou = false;
			std::string nome = "";
			for( osg::NodePath::const_iterator hitNode = path.begin(); hitNode != path.end(); ++hitNode)
			{
				if( strcmp( (*hitNode)->className(),"Geode" ) == 0 || strcmp( (*hitNode)->className(),"Terrain" ) == 0 )
				{
					nome = (*hitNode)->getName();
					achou = true;
					osg::ShapeDrawable* shpDrb;
					if ( strcmp( (*hitNode)->className(),"Geode" ) == 0 )
					{
						shpDrb = dynamic_cast<osg::ShapeDrawable*>((*hitNode)->asGroup()->getChild(0));
						if( selecionado == nullptr )
						{
							if (shpDrb)
								shpDrb->setColor( osg::Vec4(1,1,1,1) );
							selecionado = (*hitNode)->asGeode();
						}
						else
						{
							if( selecionado->getName().compare( (*hitNode)->getName() ) != 0 )
							{
								if (shpDrb)
									shpDrb->setColor( osg::Vec4(1,1,1,1) );
								shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->asGroup()->getChild(0));
								if (shpDrb)
									shpDrb->setColor( osg::Vec4(1,0,0,1) );
								selecionado = (*hitNode)->asGeode();
							}
						}
					}
					else // Terrain
					{
						if( selecionado != nullptr)
						{
							shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->asGroup()->getChild(0));
							if (shpDrb)
								shpDrb->setColor( osg::Vec4(1,0,0,1) );
							selecionado = nullptr;
						}
					}
					break;
				}
			}
			if( achou )
				return "Cursor lat:lon:alt: " + ptrX.str() + ":" + ptrY.str() + ":" + ptrZ.str() + "\n" + nome;
			else
				return "Cursor lat:lon:alt: " + ptrX.str() + ":" + ptrY.str() + ":" + ptrZ.str();
		}
	}
	else
		return "";
}

// https://groups.google.com/forum/#!searchin/osg-users/LineSegmentIntersector|sort:date/osg-users/f4WZnzr8X5w/37pAZ4QqAwAJ
// src/osgUtil/LineSegmentIntersector.cpp
float OVNIController::pickDown( osg::Vec3d pos ) // retorna altitude do terreno (className() == "Geometry") naquele ponto, ou -1 se estiver fora do terreno
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	osg::Vec3d pos1( pos.x(), pos.y(), pos.z()+100 );
	osg::Vec3d pos2( pos.x(), pos.y(), -100 );
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intsec = new osgUtil::LineSegmentIntersector( osgUtil::Intersector::MODEL, pos1, pos2 );
	osgUtil::IntersectionVisitor iv( intsec.get() );
	_viewer->getSceneData()->accept( iv );
	if( intsec->containsIntersections() )
	{
        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intsec->getIntersections().begin();
            hitr != intsec->getIntersections().end();
            ++hitr)
        {
			if ( strcmp(hitr->drawable->className(),"Geometry") == 0 )
			{
				return hitr->getWorldIntersectPoint().z();
			}
		}
		return -1;
	}
	else
		return -1;
}

// apertou o botão do mouse
bool OVNIController::handleMousePush(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    flushMouseEventStack();
	addMouseEvent( ea );
	unsigned int buttonMask = _ga_t0->getButtonMask();
	if( selecionado && mouseFree )
	{
		osg::ShapeDrawable* shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->getChild(0));
		if (shpDrb)
		{
			if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
			{
				shpDrb->setColor( osg::Vec4(1,1,0,1) ); // amarelo
				movendoBox = true;
			}
		}
	}
	return false;
}

// soltou o botão do mouse
bool OVNIController::handleMouseRelease(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    flushMouseEventStack();
	addMouseEvent( ea );
	if( selecionado )
	{
		osg::ShapeDrawable* shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->getChild(0));
		if (shpDrb)
		{
			shpDrb->setColor( osg::Vec4(1,1,1,1) ); // branco
			movendoBox = false;
		}
	}
	return false;
}

bool OVNIController::performMovement()
{
	// return if less then two events have been added
	if( _ga_t0.get() == NULL || _ga_t1.get() == NULL )
		return false;
	if( mouseFree )
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
	unsigned int modKeyMask = _ga_t1->getModKeyMask();
	unsigned int buttonMask = _ga_t1->getButtonMask();
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

void calcAcc(osgViewer::Viewer* viewer) // calcula aceleração do OVNI
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
	camSpeed.set(velX, velY, velZ);

	// gravidade
	osg::Vec3d vel, pos;
	osg::Matrix matrix;
	osg::Quat rotation;
	osg::ref_ptr<osg::MatrixTransform> bloco;
	for (int i=0; i<blocos->getNumChildren(); i++)
	{
		bloco = dynamic_cast<osg::MatrixTransform*>( blocos->getChild(i) );
		bloco->getChild(0)->getUserValue( "speed", vel );
		if ( vel != osg::Vec3d( 0,0,0 ) )
		{
			vel += osg::Vec3d( 0,0,-0.005 ); // acelera
			matrix = bloco->getMatrix();
			rotation = matrix.getRotate();
			pos = matrix.getTrans();
			pos += rotation*vel;
			float chao = controller->pickDown( pos ) + 0.05; // 0.5 é relativo ao tamanho do cubo
			if ( pos.z() <= chao ) // para no chão
			{
				vel = osg::Vec3d( 0,0,0 );
				pos.z() = chao;
				bloco->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
			}
			else // continua caindo
			{
				bloco->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
			}
			bloco->getChild(0)->setUserValue( "speed", vel );
		}
	}
	// Sol
	bool solMudou = false;
	if (keyL)
	{
		solMudou = true;
		solLO += velSol;
		if( solLO > pi)
			solLO -= 2*pi;
	}
	else
	if (keyO)
	{
		solMudou = true;
		solLO -= velSol;
		if( solLO < pi)
			solLO += 2*pi;
	}
	if (keyI)
	{
		solMudou = true;
		solNS += velSol;
		if( solNS > pi)
			solNS -= 2*pi;
	}
	else
	if (keyK)
	{
		solMudou = true;
		solNS -= velSol;
		if( solNS < pi)
			solNS += 2*pi;
	}
	if (solMudou)
	{
		posSol = osg::Vec3d( lonC+rCeu*sin(solLO), latC+rCeu*sin(solNS), rCeu*cos(solLO)*cos(solNS) );
		osg::MatrixTransform* SolM = dynamic_cast<osg::MatrixTransform*>( Sol.get() );
		SolM->setMatrix( osg::Matrix::translate( posSol ) );
	}
}

void criaMenus() // examples/osggeometry/osggeometry.cpp
{
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

osg::Node* createLightSource( unsigned int num, const osg::Vec3& trans, const osg::Vec4& color )
{
	osg::ref_ptr<osg::Light> light = new osg::Light;
	light->setLightNum( num );
	light->setDiffuse( color );
	light->setPosition( osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
	lightSource->setLight( light );
	osg::ref_ptr<osg::MatrixTransform> sourceTrans = new osg::MatrixTransform;
	sourceTrans->setMatrix( osg::Matrix::translate(trans) );
	sourceTrans->addChild( lightSource.get() );
	return sourceTrans.release();
}

int main( int argc, char** argv )
{
	blocos = new osg::Group;
	osg::ref_ptr<osg::Group> root = new osg::Group;
	osg::ref_ptr<osg::Node> BRnode = osgDB::readNodeFile("BR.osgb");
	BRnode->setName("Terreno");
	root->addChild( BRnode );
	root->addChild( blocos );
	// Luz
	Sol = createLightSource( 0, posSol, osg::Vec4(0.8f,0.8f,0.8f,1.0f) );
	root->getOrCreateStateSet()->setMode( GL_LIGHT0, osg::StateAttribute::ON );
	root->addChild( Sol );
	// Viewer
	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	//osg::ref_ptr<OVNIController> controller = new OVNIController(&viewer);
	controller = new OVNIController(&viewer);
	viewer.setCameraManipulator(controller);
	if( argc > 1 && strcmp( argv[1], "j" ) == 0 )
		viewer.setUpViewInWindow(0, 0, 800, 600); // deprecated? osgViewer::View::apply( ViewConfig* config )
	viewer.realize();
	// Posição inicial da câmera
	osg::Quat quad0;
	controller->setTransformation(osg::Vec3(lonC,latC,rCeu), quad0 ); // camLon (-74-34.8)/2 = -54.4, camLat (5.3333333-33.8666667)/2 = -14.2666667, camAlt = 80
	// Detecta coordenadas da janela
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	window = windows.begin();
	(*window)->useCursor(false);
	(*window)->getWindowRectangle(windowX, windowY, windowW, windowH);
	std::cout << "Tamanho da janela: " << windowW << "x" << windowH << "\n";
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
	//text = createText(osg::Vec3(10, windowH-30, 0), "", 25);
	text = createText(osg::Vec3(10, windowH-30, 0), "", windowW/64);
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
		calcAcc( &viewer );
		if (Homing) // volta a câmera à posição inicial ao apertar Home
		{
			controller->setTransformation(osg::Vec3(lonC,latC,rCeu), quad0 );
			Homing = false;
		}
		viewer.frame();
	}
	return 0;
}

// g++ terreno1.cpp -losg -losgDB -losgFX -losgGA -losgText -losgUtil -losgViewer -o terreno1

// g++ terreno1.cpp -Wl,-Bstatic -losg -losgDB -losgGA -losgText -losgUtil -losgViewer -o terreno1 (ainda não funciona)
