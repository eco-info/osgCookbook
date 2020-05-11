// Time for action – driving the Cessna

#include <iostream>
#include <string>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgText/Text>

using namespace std;

namespace osgCookbook {
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
}

osgViewer::Viewer viewer;
osgText::Text* text;

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

class ModelController : public osgGA::GUIEventHandler
{
public:
	ModelController( osg::MatrixTransform* node ) : _model(node) {}
	virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );
protected:
	osg::ref_ptr<osg::MatrixTransform> _model;
};

bool ModelController::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
	if ( !_model )
		return false;
	osg::Matrix matrix = viewer.getCamera()->getViewMatrix();
	switch ( ea.getEventType() )
	{
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch ( ea.getKey() )
			{
				case 'a': case 'A':
					matrix *= osg::Matrix::rotate(-0.1f, osg::Y_AXIS);
					break;
				case 'd': case 'D':
					matrix *= osg::Matrix::rotate(0.1f, osg::Y_AXIS);
					break;
				case 'w': case 'W':
					matrix *= osg::Matrix::rotate(-0.1f, osg::X_AXIS);
					break;
				case 's': case 'S':
					matrix *= osg::Matrix::rotate(0.1f, osg::X_AXIS);
					break;
				default:
					break;
			}
			viewer.getCamera()->setViewMatrix( matrix );
			break;
		default:
			break;
	}
	osg::Vec3 hpr = fromQuat(matrix.getRotate(),true); // ângulos da câmera, em graus
	double h = hpr.x();
	double p = hpr.y();
	double r = hpr.z();
	text->setText( to_string(h) + "|" + to_string(p) + "|" + to_string(r) );
	return false;
}

int main( int argc, char** argv )
{
	// Create the text and place it in an HUD camera
	text = osgCookbook::createText(osg::Vec3(50.0f, 50.0f, 0.0f), "0.000000|-90.000000|0.000000", 10.0f);
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	textGeode->addDrawable( text );
	osg::ref_ptr<osg::Camera> hudCamera = osgCookbook::createHUDCamera(0, 800, 0, 600);
	hudCamera->addChild( textGeode.get() );
	
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile( "cessna.osg" );
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->addChild( model.get() );
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( hudCamera.get() );
	root->addChild( mt.get() );
	
	osg::ref_ptr<ModelController> ctrler = new ModelController( mt.get() );
	
	viewer.addEventHandler( ctrler.get() );
	viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
	viewer.getCamera()->setAllowEventFocus( false );
	
	viewer.setSceneData( root.get() );
	//return viewer.run();
	viewer.realize();
	while ( !viewer.done() )
	{
		viewer.frame();
	}
	return 0;
}

// g++ beg3.234.cpp -losg -losgDB -losgGA -losgText -losgViewer -o beg3.234
