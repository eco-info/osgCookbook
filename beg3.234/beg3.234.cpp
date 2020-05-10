// Time for action – driving the Cessna

#include <iostream>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

using namespace std;

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
    if(degrees)
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
	/* Ângulos da câmera:
	osg::Matrixd mx = view->getCamera()->getInverseViewMatrix();
	osg::Vec3 hpr = fromQuat(mx.getRotate(),false);
	h = osg::RadiansToDegrees(hpr.x());
	p = osg::RadiansToDegrees(hpr.y())-90.0;
	r = osg::RadiansToDegrees(hpr.z());*/
	osg::Matrix matrix = _model->getMatrix();
	osg::Vec3 hpr = fromQuat(matrix.getRotate(),true); // ângulos do Cessna, em graus
	double h = hpr.x();
	double p = hpr.y()-90.0;
	double r = hpr.z();
	switch ( ea.getEventType() )
	{
		case osgGA::GUIEventAdapter::KEYDOWN:
			cout << (char)ea.getKey() << ": " << h << "|" << p << "|" << r << "\n";
			switch ( ea.getKey() )
			{
				case 'a': case 'A':
					matrix *= osg::Matrix::rotate(-0.1f, osg::Z_AXIS);
					break;
				case 'd': case 'D':
					matrix *= osg::Matrix::rotate(0.1f, osg::Z_AXIS);
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
			_model->setMatrix( matrix );
			break;
		default:
			break;
	}
	return false;
}

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile( "cessna.osg" );
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->addChild( model.get() );
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild( mt.get() );
	
	osg::ref_ptr<ModelController> ctrler = new ModelController( mt.get() );
	
	osgViewer::Viewer viewer;
	viewer.addEventHandler( ctrler.get() );
	viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
	viewer.getCamera()->setAllowEventFocus( false );
	
	viewer.setSceneData( root.get() );
	return viewer.run();
}

// g++ beg3.234.cpp -losg -losgDB -losgGA -losgViewer -o beg3.234
