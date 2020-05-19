// Creating a simple Galaxian game

#include <osg/Texture2D>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
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
}

class Player : public osg::MatrixTransform
{
public:
	Player() : _type(INVALID_OBJ) {}
	Player( float width, float height, const std::string& texfile );
	float width() const { return _size[0]; }
	float height() const { return _size[1]; }
	void setSpeedVector( const osg::Vec3& sv )
	{
		_speedVec = sv;
	}
	const osg::Vec3& getSpeedVector() const
	{
		return _speedVec;
	}
	enum PlayerType
	{
		INVALID_OBJ=0, PLAYER_OBJ, ENEMY_OBJ, PLAYER_BULLET_OBJ, ENEMY_BULLET_OBJ
	};
	void setPlayerType( PlayerType t ) { _type = t; }
	PlayerType getPlayerType() const { return _type; }
	bool isBullet() const
	{
		return _type==PLAYER_BULLET_OBJ || _type==ENEMY_BULLET_OBJ;
	}
	bool update( const osgGA::GUIEventAdapter& ea, osg::Group* root );
	bool intersectWith( Player* player ) const;
protected:
	osg::Vec2 _size;
	osg::Vec3 _speedVec;
	PlayerType _type;
};

Player::Player( float width, float height, const std::string& texfile ) : _type(INVALID_OBJ)
{
	_size.set( width, height );
	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	texture->setImage( osgDB::readImageFile(texfile) );
	osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry(
		osg::Vec3(-width*0.5f, -height*0.5f, 0.0f),
		osg::Vec3(width, 0.0f, 0.0f), osg::Vec3(0.0f, height, 0.0f) );
	quad->getOrCreateStateSet()->setTextureAttributeAndModes( 0, texture.get() );
	quad->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	quad->getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( quad.get() );
	addChild( geode.get() );
}

bool Player::update( const osgGA::GUIEventAdapter& ea, osg::Group* root )
{
	bool emitBullet = false;
	switch ( _type )
	{
		case PLAYER_OBJ:
			if ( ea.getEventType()==osgGA::GUIEventAdapter::KEYDOWN )
			{
				switch ( ea.getKey() )
				{
					case osgGA::GUIEventAdapter::KEY_Left:
						_speedVec = osg::Vec3(-0.1f, 0.0f, 0.0f);
						break;
					case osgGA::GUIEventAdapter::KEY_Right:
						_speedVec = osg::Vec3(0.1f, 0.0f, 0.0f);
						break;
					case osgGA::GUIEventAdapter::KEY_Return:
						emitBullet = true;
						break;
					default: break;
				}
			}
			else
			if ( ea.getEventType()==osgGA::GUIEventAdapter::KEYUP )
				_speedVec = osg::Vec3();
			break;
		case ENEMY_OBJ:
			if ( RAND(0, 2000)<1 )
				emitBullet = true;
			break;
		default: break;
	}

	osg::Vec3 pos = getMatrix().getTrans();
	if ( emitBullet )
	{
		osg::ref_ptr<Player> bullet = new Player( 0.4f, 0.8f, "bullet.png" );
		if ( _type==PLAYER_OBJ )
		{
			bullet->setPlayerType( PLAYER_BULLET_OBJ );
			bullet->setMatrix( osg::Matrix::translate( pos + osg::Vec3(0.0f, 0.9f, 0.0f)) );
			bullet->setSpeedVector( osg::Vec3(0.0f, 0.2f, 0.0f) );
		}
		else
		{
			bullet->setPlayerType( ENEMY_BULLET_OBJ );
			bullet->setMatrix( osg::Matrix::translate( pos - osg::Vec3(0.0f, 0.9f, 0.0f)) );
			bullet->setSpeedVector( osg::Vec3(0.0f,-0.2f, 0.0f) );
		}
		root->addChild( bullet.get() );
	}

	if ( ea.getEventType()!=osgGA::GUIEventAdapter::FRAME )
		return true;
	float halfW = width() * 0.5f, halfH = height() * 0.5f;
	pos += _speedVec;
	// Don't update the player anymore if it is not in the visible area.
	if ( pos.x()<halfW || pos.x()>ea.getWindowWidth()-halfW )
		return false;
	if ( pos.y()<halfH || pos.y()>ea.getWindowHeight()-halfH )
		return false;
	setMatrix( osg::Matrix::translate(pos) );
	return true;
}

bool Player::intersectWith( Player* player ) const
{
	osg::Vec3 pos = getMatrix().getTrans();
	osg::Vec3 pos2 = player->getMatrix().getTrans();
	return fabs(pos[0] - pos2[0]) < (width() + player->width()) * 0.5f &&
	fabs(pos[1] - pos2[1]) < (height() + player->height()) * 0.5f;
}

class GameControllor : public osgGA::GUIEventHandler
{
public:
	GameControllor( osg::Group* root ) : _root(root), _direction(0.1f), _distance(0.0f) {}
	virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );
protected:
	osg::observer_ptr<osg::Group> _root;
	float _direction;
	float _distance;
};

bool GameControllor::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
	_distance += fabs(_direction);
	if ( _distance>30.0f )
	{
		_direction = -_direction;
		_distance = 0.0f;
	}

	osg::NodePath toBeRemoved;
	for ( unsigned i=0; i<_root->getNumChildren(); ++i )
	{
		Player* player = static_cast<Player*>( _root->getChild(i) );
		if ( !player )
			continue;
		// Update the player matrix, and remove the player if it is a bullet outside the visible area
		if ( !player->update(ea, _root.get()) )
		{
			if ( player->isBullet() )
				toBeRemoved.push_back( player );
		}
		// Automatically move the enemies
		if ( player->getPlayerType()==Player::ENEMY_OBJ )
			player->setSpeedVector( osg::Vec3(_direction, 0.0f, 0.0f) );
		if ( !player->isBullet() )
			continue;
		// Use a simple loop to check if any two of the players (you, enemies, and bullets) are intersected
		for ( unsigned j=0; j<_root->getNumChildren(); ++j )
		{
			Player* player2 = static_cast<Player*>( _root->getChild(j) );
			if ( !player2 || player==player2 )
				continue;
			if ( player->getPlayerType()==Player::ENEMY_BULLET_OBJ && player2->getPlayerType()==Player::ENEMY_OBJ )
			{
				continue;
			}
			else
			if ( player->intersectWith(player2) )
			{
				// Remove both players if they collide with each other
				toBeRemoved.push_back( player );
				toBeRemoved.push_back( player2 );
			}
		}
	}

	for ( unsigned i=0; i<toBeRemoved.size(); ++i )
		_root->removeChild( toBeRemoved[i] );
	return false;
}

int main( int argc, char** argv )
{
	osg::ref_ptr<Player> player = new Player( 1.0f, 1.0f, "player.png" );
	player->setMatrix( osg::Matrix::translate(40.0f, 5.0f, 0.0f) );
	player->setPlayerType( Player::PLAYER_OBJ );
	osg::ref_ptr<osg::Camera> hudCamera = osgCookBook::createHUDCamera(0, 80, 0, 30);
	hudCamera->addChild( player.get() );

	for ( unsigned int i=0; i<5; ++i )
	{
		for ( unsigned int j=0; j<10; ++j )
		{
			osg::ref_ptr<Player> enemy = new Player(1.0f, 1.0f, "enemy.png");
			enemy->setMatrix( osg::Matrix::translate(20.0f+1.5f*(float)j, 25.0f-1.5f*(float)i, 0.0f) );
			enemy->setPlayerType( Player::ENEMY_OBJ );
			hudCamera->addChild( enemy.get() );
		}
	}

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor( osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
	viewer.addEventHandler( new GameControllor(hudCamera.get()) );
	viewer.setSceneData( hudCamera.get() );
	return viewer.run();
}

// g++ cook5.8.cpp -losg -losgDB -losgGA -losgViewer -o cook5.8
