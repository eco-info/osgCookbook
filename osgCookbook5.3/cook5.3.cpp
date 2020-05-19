// Designing scrolling text

#include <osgAnimation/EaseMotion>
#include <osgDB/ReadFile>
#include <osgText/Text>
#include <osgViewer/Viewer>
#include <sstream>
#include <iomanip>
#define RAND(min, max) ((min) + (float)rand()/(RAND_MAX) * ((max)-(min)))

namespace osgCookBook {
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
	osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("Charter.ttf");
	osgText::Text* createText( const osg::Vec3& pos, const std::string& content, float size )
	{
		osg::ref_ptr<osgText::Text> text = new osgText::Text;
		text->setDataVariance( osg::Object::DYNAMIC );
		text->setFont( g_font.get() );
		text->setCharacterSize( size );
		text->setAxisAlignment( osgText::TextBase::XY_PLANE );
		text->setPosition( pos );
		text->setText( content );
		return text.release();
	}
}

class ScrollTextCallback : public osg::Drawable::UpdateCallback
{
public:
	ScrollTextCallback()
	{
		_motion = new osgAnimation::LinearMotion;
		computeNewPosition();
	}
	virtual void update( osg::NodeVisitor* nv, osg::Drawable* drawable );
	void computeNewPosition()
	{
		_motion->reset();
		_currentPos.y() = RAND(50.0, 500.0);
	}
protected:
	osg::ref_ptr<osgAnimation::LinearMotion> _motion;
	osg::Vec3 _currentPos;
};

void ScrollTextCallback::update( osg::NodeVisitor* nv, osg::Drawable* drawable )
{
	osgText::Text* text = static_cast<osgText::Text*>( drawable );
	if ( !text )
		return;
	_motion->update( 0.002 );
	float value = _motion->getValue();
	if ( value>=1.0f )
		computeNewPosition();
	else
		_currentPos.x() = value * 800.0f;
	std::stringstream ss; ss << std::setprecision(3);
	ss << "XPos: " << std::setw(5) << std::setfill(' ') << _currentPos.x() << "; YPos: " << std::setw(5) << std::setfill(' ') << _currentPos.y();
	text->setPosition( _currentPos );
	text->setText( ss.str() );
}

int main( int argc, char** argv )
{
	osgText::Text* text = osgCookBook::createText(osg::Vec3(), "", 20.0f);
	text->addUpdateCallback( new ScrollTextCallback );
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	textGeode->addDrawable( text );
	osg::ref_ptr<osg::Camera> hudCamera = osgCookBook::createHUDCamera(0, 800, 0, 600);
	hudCamera->addChild( textGeode.get() );

	osgViewer::Viewer viewer;
	viewer.setSceneData( hudCamera.get() );
	return viewer.run();
}

// g++ cook5.3.cpp -losg -losgAnimation -losgDB -losgViewer -losgText -o cook5.3
