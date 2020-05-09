// Designing a 2D camera manipulator

#include <osgDB/ReadFile>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

class TwoDimManipulator : public osgGA::StandardManipulator
{
public:
	TwoDimManipulator() : _distance(1.0) {}
	
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix( const osg::Matrixd& matrix );
	virtual void setByInverseMatrix( const osg::Matrixd& matrix );
	
	// Leave empty as we don't need these here. They are used by other functions and classes to set up the manipulator directly.
	virtual void setTransformation( const osg::Vec3d&, const osg::Quat& ) {}
	virtual void setTransformation( const osg::Vec3d&, const osg::Vec3d&, const osg::Vec3d& ) {}
	virtual void getTransformation( osg::Vec3d&, osg::Quat& ) const {}
	virtual void getTransformation( osg::Vec3d&, osg::Vec3d&, osg::Vec3d& ) const {}
	virtual void home( double );
	virtual void home( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	virtual ~TwoDimManipulator() {}
	virtual bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );
	virtual bool performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy );
	osg::Vec3 _center;
	double _distance;
};

osg::Matrixd TwoDimManipulator::getMatrix() const
{
	osg::Matrixd matrix;
	matrix.makeTranslate( 0.0f, 0.0f, _distance );
	matrix.postMultTranslate( _center );
	return matrix;
}

osg::Matrixd TwoDimManipulator::getInverseMatrix() const
{
	osg::Matrixd matrix;
	matrix.makeTranslate( 0.0f, 0.0f,-_distance );
	matrix.preMultTranslate( -_center );
	return matrix;
}

void TwoDimManipulator::setByMatrix( const osg::Matrixd& matrix )
{
	setByInverseMatrix( osg::Matrixd::inverse(matrix) );
}

void TwoDimManipulator::setByInverseMatrix( const osg::Matrixd& matrix )
{
	osg::Vec3d eye, center, up;
	matrix.getLookAt( eye, center, up );
	_center = center; _center.z() = 0.0f;
	if ( _node.valid() )
		_distance = abs((_node->getBound().center() - eye).z());
	else
		_distance = abs((eye - center).length());
}

void TwoDimManipulator::home( double )
{
	if ( _node.valid() )
	{
		_center = _node->getBound().center();
		_center.z() = 0.0f;
		_distance = 2.5 * _node->getBound().radius();
	}
	else
	{
		_center.set( osg::Vec3() );
		_distance = 1.0;
	}
}

void TwoDimManipulator::home( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
	home( ea.getTime() );
}

bool TwoDimManipulator::performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	_center.x() -= 100.0f * dx;
	_center.y() -= 100.0f * dy;
	return false;
}

bool TwoDimManipulator::performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	_distance *= (1.0 + dy);
	if ( _distance<1.0 )
		_distance = 1.0;
	return false;
}

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( osgDB::readNodeFile("lz.osg") );
	
	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keySwitch = new osgGA::KeySwitchMatrixManipulator;
	keySwitch->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator );
	keySwitch->addMatrixManipulator( '2', "TwoDim", new TwoDimManipulator );
	const osg::BoundingSphere& bs = root->getBound();
	keySwitch->setHomePosition( bs.center()+osg::Vec3(0.0f, 0.0f, bs.radius()), bs.center(), osg::Y_AXIS );
	
	osgViewer::Viewer viewer;
	viewer.setCameraManipulator( keySwitch.get() );
	viewer.setSceneData( root.get() );
	return viewer.run();
}

// g++ cook4.8.cpp -losg -losgDB -losgGA -losgViewer -o cook4.8
