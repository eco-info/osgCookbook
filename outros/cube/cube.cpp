// https://github.com/SunilRao01/OSGFirstPersonController

#include <osg/ShapeDrawable>
#include <osgGA/FirstPersonManipulator>
#include <osgViewer/Viewer>
#include <iostream>

// FirstPersonController.h
class FirstPersonController : public osgGA::FirstPersonManipulator
{
public:
   FirstPersonController(osgViewer::Viewer *inputViewer) : _viewer(inputViewer)
   {
       // Start frame timer
       mainTimer.setStartTick();
       setAllowThrow(false); // não continua movendo ao soltar o botão do mouse
   }
   virtual bool performMovement();
   // Override handle function for keyboard input
   virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &);
protected:
   osgViewer::Viewer *_viewer;
   osg::Timer mainTimer;
};

// FirstPersonController.cpp
const float moveSpeed = 0.6;
const float inputTimeInterval = 0.02;
osg::Vec3d tempMov;
double maxTick = inputTimeInterval;

bool FirstPersonController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    // Still use first person manipulator for camera movements (Inherited class function)
    FirstPersonManipulator::handle(ea, aa);
   if (!_viewer)
   {
       return false;
   }
   // Set the viewer's "eye" position, which is located at the center of the camera.
   osg::Vec3d eyePos;
   osg::Matrix matrix = _viewer->getCameraManipulator()->getMatrix();
   eyePos = matrix.getTrans();
   osg::Quat camRotation = matrix.getRotate();
   switch(ea.getEventType())
   {
      case(osgGA::GUIEventAdapter::KEYDOWN):
      {
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

                 break;
             default:
                 break;
         }
        default:
         break;
      }
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
                   break;
               default:
                   break;
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

bool FirstPersonController::performMovement()
{
    // return if less then two events have been added
    if( _ga_t0.get() == NULL || _ga_t1.get() == NULL )
    {
        return false;
    }
    // get delta time, throw warning if error with delta time
    double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
    if( eventTimeDelta < 0. )
    {
        OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
        eventTimeDelta = 0.;
    }
    // get deltaX and deltaY
    float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
    float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();
    std::cout << dx << ", " << dy << "\n";
    // return if there is no movement.
    if( dx == 0.0 && dy == 0.0 )
    {
        return false;
    }
	performMouseDeltaMovement(dx, dy);
    // call appropriate methods -- se chama um dos de baixo, apenas duplica (i.e. acelera) o movimento já feito na linha acima
    /*unsigned int buttonMask = _ga_t1->getButtonMask();
    if( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
    {
        return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
    }
    else if( buttonMask == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
            buttonMask == (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) )
    {
        return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
    }
    else if( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
    {
        return performMovementRightMouseButton( eventTimeDelta, dx, dy );
    }*/

    return false;
}

// cube.cpp

int main( int argc, char** argv )
{
    //Create ROOT
    osg::ref_ptr<osg::Group> root (new osg::Group);
    //Create BOX
    osg::ref_ptr<osg::Box> myBox (new osg::Box(osg::Vec3(1, 1, 1), 1));
    osg::ref_ptr<osg::ShapeDrawable> boxDrawable (new osg::ShapeDrawable(myBox.get()));
    osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
    myshapegeode->addDrawable(boxDrawable.get());
    // Add cube
    root->addChild(myshapegeode);
    osgViewer::Viewer viewer;
    viewer.setSceneData(root.get());
    // KEYBOARD INPUT
    osg::ref_ptr<FirstPersonController> controller = new FirstPersonController(&viewer);
    viewer.setCameraManipulator(controller);
    viewer.setUpViewInWindow(200, 200, 640, 480);
    viewer.realize();
    while (!viewer.done())
    {
        viewer.frame();
    }
    return 0;
}

// g++ cube.cpp -losg -losgGA -losgViewer -o cube
