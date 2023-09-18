#include <iostream>
#include <qdebug.h>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/LineWidth>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgAnimation/Sampler>
#include <osgText/Text>
#include <osgDB/WriteFile>
#include <osgViewer/GraphicsWindow>
#include <thread>
#include <osgUtil/RenderBin>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgParticle/FireEffect>
#include <osgParticle/SmokeEffect>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/AccelOperator>
#include <osgParticle/ExplosionEffect>
#include <osgParticle/ExplosionDebrisEffect>
#include <osgParticle/PrecipitationEffect>
#include <osgFX/Outline>
#include <osgFX/Scribe>
#include <osg/Fog>

osg::Group* root = new osg::Group();


class AnimtkUpdateCallback : public osg::NodeCallback
{
public:
    META_Object(osgAnimation, AnimtkUpdateCallback);


    AnimtkUpdateCallback()
    {
        _sampler = new osgAnimation::Vec3CubicBezierSampler;
        _playing = false;
        _lastUpdate = 0;
    }
    AnimtkUpdateCallback(const AnimtkUpdateCallback& val, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY):
        osg::Object(val, copyop),
        osg::Callback(val, copyop),
        osg::NodeCallback(val, copyop),
        _sampler(val._sampler),
        _startTime(val._startTime),
        _currentTime(val._currentTime),
        _playing(val._playing),
        _lastUpdate(val._lastUpdate)
    {
    }

public:
    /** Callback method called by the NodeVisitor when visiting a node.*/
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR &&
            nv->getFrameStamp() &&
            nv->getFrameStamp()->getFrameNumber() != _lastUpdate)
        {
            _lastUpdate = nv->getFrameStamp()->getFrameNumber();
            _currentTime = osg::Timer::instance()->tick();
            if (_playing && _sampler.get() && _sampler->getKeyframeContainer())
            {
                osg::PositionAttitudeTransform* transform = dynamic_cast<osg::PositionAttitudeTransform*>(node);
                if (transform) {
                    osg::Vec3 result;
                    float t = osg::Timer::instance()->delta_s(_startTime, _currentTime);
                    float duration = _sampler->getEndTime() - _sampler->getStartTime();
                    t = fmod(t, duration);
                    t += _sampler->getStartTime();
                    _sampler->getValueAt(t, result);
                    transform->setPosition(result);
                    osg::Vec3d rotVec =    result - lastPosition;
                    osg::Vec3d headVec = osg::Vec3d(0,1,0);
                    rotVec = rotVec/rotVec.length();
                    osg::Quat headingRotate;
                    headingRotate.makeRotate(headVec, rotVec);
                    transform->setAttitude(headingRotate);
                    lastPosition = result;
                }
            }
        }
        traverse(node,nv);
    }

    void start() { _startTime = osg::Timer::instance()->tick(); _currentTime = _startTime; _playing = true;}
    void stop() { _currentTime = _startTime; _playing = false;}

    osg::ref_ptr<osgAnimation::Vec3CubicBezierSampler> _sampler;
    osg::Timer_t _startTime;
    osg::Timer_t _currentTime;
    bool _playing;
    unsigned int _lastUpdate;
    osg::Vec3 lastPosition;
    osg::ref_ptr<osg::Geode> _geode;
};



class MakePathDistanceCallback: public AnimtkUpdateCallback
{
    osg::Vec3 _lastAdd;
    float _threshold;
    unsigned int _count;
public:
    MakePathDistanceCallback(osg::Geode* geode):
        _threshold(0.5f),
        _count(0) {_geode = geode;}
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        static bool countReported = false;
        float t = osg::Timer::instance()->delta_s(_startTime, _currentTime);
        osg::Vec3 pos;
        _sampler->getValueAt(t, pos);
        osg::Vec3 distance = _lastAdd - pos;
//        if(t <= 15.0f && distance.length() >= _threshold)
//        {
//            _geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(pos, 0.3f)));
//            _lastAdd = pos;
//            _count++;
//        }
//        else if(t > 15.0f)
//        {
//            if(!countReported) std::cout << "Created " << _count << " nodes." << std::endl;
//            countReported = true;
//        }
        AnimtkUpdateCallback::operator()(node, nv);
    }
};





osg::PositionAttitudeTransform* setupAnimtkNode(osg::Geode* staticGeode)
{
    ////////////////////////////////////////////////////////////////////
    osg::Vec3d objectDirection  = osg::Vec3d(-1,1,0);  //init direction of object
    double objectLength = 4;                          //lenght of object
    osg::Vec3d startPoint = osg::Vec3d(0,0,0);       // current object position
    osg::Vec3d end        = osg::Vec3d(100,0,50);   //object distination
    /////////////////////////////////////////////////////////////////
    osg::Vec3d linePathVector = end - startPoint ;
    double distance = linePathVector.length();
    osg::Vec3d pathDirectionVector = linePathVector / distance;
    osg::Vec3d middle_vector = (pathDirectionVector + objectDirection) / (sqrt(1 - (pathDirectionVector * objectDirection)));
    osg::Vec3d point2_Direction = (middle_vector + objectDirection) / (sqrt(abs(1 - ((middle_vector/middle_vector.length()) * objectDirection))));
    double angle = acos(pathDirectionVector*objectDirection) * 180 / 3.14;
    std::cout << angle;
    ////////////////////////////////////////////////////////////////
    osg::Vec3d point1 ;  //osg::Vec3d(50,50,2); // start point of making arc
    point1.x() = startPoint.x() + 1*objectLength*objectDirection.x();
    point1.y() = startPoint.y() + 1*objectLength*objectDirection.y();
    point1.z() = startPoint.z() + 1*objectLength*objectDirection.z();
    ////////////////////////////////////////////////////////////////
    osg::Vec3d Point3; //= osg::Vec3d(0,50,50);
    Point3.x() = end.x() - 20*objectLength*(pathDirectionVector.x());
    Point3.y() = end.y() - 20*objectLength*(pathDirectionVector.y());
    Point3.z() = end.z(); /*- 3*objectLength*(-pathDirectionVector.z());*/
    ////////////////////////////////////////////////////////////////
    osg::Vec3d point2;
    osg::Vec3d rotototo = objectDirection - pathDirectionVector;
    point2.x() = startPoint.x() + 10*objectLength * rotototo.x() - 10*objectDirection.x();
    point2.y() = startPoint.y() + 10*objectLength * rotototo.y() - 10*objectDirection.y();
    point2.z() = startPoint.z() ;

    if(165 < angle && angle < 195){
        point2.x() = startPoint.x() + 10*objectLength*objectDirection.x() ;
        point2.y() = startPoint.y() + 10*objectLength ;
    }

    osg::PositionAttitudeTransform* node = new osg::PositionAttitudeTransform();
    AnimtkUpdateCallback* callback = new MakePathDistanceCallback(staticGeode);
    osgAnimation::Vec3CubicBezierKeyframeContainer* keys = callback->_sampler->getOrCreateKeyframeContainer();
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(0, osgAnimation::Vec3CubicBezier(
                                                                 startPoint,point2,Point3
                                                                 )));

    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(10, osgAnimation::Vec3CubicBezier(
                                                                  end
                                                                  )));

    callback->start();
    node->setUpdateCallback(callback);
//    osg::ref_ptr<osg::Node> airPlane =  osgDB::readNodeFile("./boeing-747.osgb");
//    node->setScale(osg::Vec3d(0.2,0.2,0.2));
//    node->addChild(airPlane);

    ///////////////////////////////////////







    //particle////////////////
    osg::ref_ptr<osgParticle::SmokeEffect> mFire1;
    mFire1 = new osgParticle::SmokeEffect(osg::Vec3(11,0,0),1,2);
    mFire1->setUseLocalParticleSystem(false);
    mFire1->setParticleDuration(0.5);
    mFire1->setTextureFileName("continous_smoke.rgb");
    node->addChild(mFire1);
    root->addChild(mFire1->getParticleSystem());

    osg::ref_ptr<osgParticle::SmokeEffect> mFire2;
    mFire2 = new osgParticle::SmokeEffect(osg::Vec3(-11,0,0),1,2);
    mFire2->setUseLocalParticleSystem(false);
    mFire2->setParticleDuration(0.5);
    mFire2->setTextureFileName("continous_smoke.rgb");
    node->addChild(mFire2);
    root->addChild(mFire2->getParticleSystem());

    osg::ref_ptr<osgParticle::SmokeEffect> mFire3;
    mFire3 = new osgParticle::SmokeEffect(osg::Vec3(22,-6,0),1,1);
    mFire3->setUseLocalParticleSystem(false);
    mFire3->setParticleDuration(1);
    mFire3->setTextureFileName("continous_smoke.rgb");
    node->addChild(mFire3);
    root->addChild(mFire3->getParticleSystem());

    osg::ref_ptr<osgParticle::SmokeEffect> mFire4;
    mFire4 = new osgParticle::SmokeEffect(osg::Vec3(-22,-6,0),1,1);
    mFire4->setUseLocalParticleSystem(false);
    mFire4->setParticleDuration(1);
    mFire4->setTextureFileName("continous_smoke.rgb");
    node->addChild(mFire4);
    root->addChild(mFire4->getParticleSystem());








    //////end particle ////////////////////////


    return node;
}



int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);
    osgGA::TrackballManipulator* tbm = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator(tbm);
//    viewer.addEventHandler(new osgViewer::StatsHandler());
//    viewer.addEventHandler(new osgViewer::WindowSizeHandler());

//    osg::Geode* geode = new osg::Geode();
//    ////////////////// add planeI///////////////////////////////////////////
//    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
//    vertices->push_back( osg::Vec3(0.0f, 0.0f, 0.0f) );
//    vertices->push_back( osg::Vec3(100.0f, 0.0f, 0.0f) );
//    vertices->push_back( osg::Vec3(100.0f, 100.0f, 0.0f) );
//    vertices->push_back( osg::Vec3(0.0f, 100.0f, 0.0f) );
//    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
//    normals->push_back( osg::Vec3(0.0f,-1.0f, 0.0f) );
//    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//    colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
//    osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
//    quad->setVertexArray( vertices.get() );
//    quad->setNormalArray( normals.get() );
//    quad->setNormalBinding( osg::Geometry::BIND_OVERALL );
//    quad->setColorArray( colors.get() );
//    quad->setColorBinding( osg::Geometry::BIND_OVERALL );
//    quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );
//    geode->addDrawable( quad.get() );
//    ////////////////////////////////////////////////plane end///////////////
//    osg::ref_ptr<osg::Node> airPlane =  osgDB::readNodeFile("./boeing-747.osgb");
//    //geode->setScale(osg::Vec3d(0.2,0.2,0.2));
//    geode->addChild(airPlane);

//    ///////////////////////////////////////

//    osg::ref_ptr<osgFX::Scribe> abbas2 = new osgFX::Scribe;
////    osg::ref_ptr<osgFX::Outline> abbas = new osgFX::Outline;
////    viewer.getCamera()->setClearStencil(0);
////    abbas->setColor(osg::Vec4f(1,0,0,1));
////    abbas->setWidth(3);
//    //abbas->dirtyBound();
//    abbas2->setWireframeColor(osg::Vec4f(1,0,0,1));

//    osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);

//    viewer.getCamera()->setClearMask(
//        GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT
//        );
//    viewer.getCamera()->setClearStencil(0);
////    abbas->addChild(airPlane.get());
//    abbas2->addChild(airPlane.get());


//    osgParticle::PrecipitationEffect *mFogEffect = new osgParticle::PrecipitationEffect;
//    mFogEffect = new osgParticle::PrecipitationEffect;
//    mFogEffect->snow(0.5);
//    mFogEffect->setUseFarLineSegments(true);
//    mFogEffect->getFog()->setStart(5);
//    mFogEffect->getFog()->setEnd(50);
////    mFogEffect->getFog()->setDensity(10.0f);
//    mFogEffect->getFog()->setMode(osg::Fog::EXP);
//    mFogEffect->getFog()->setFogCoordinateSource(osg::Fog::FRAGMENT_DEPTH);
//    geode->addChild(mFogEffect);
//    geode->getOrCreateStateSet()->setAttributeAndModes(mFogEffect->getFog());



//    geode->addChild(abbas2);
//    root->addChild(geode);
//    root->addChild(setupAnimtkNode(geode));

//    osg::Group* gp1 = new osg::Group();
//    osg::Group* gp2 = new osg::Group();

    osg::Geode* gp1 = new osg::Geode();
    osg::Geode* gp2 = new osg::Geode();




        osg::ref_ptr<osg::Node> airPlane =  osgDB::readNodeFile("./boeing-747.osgb");
//     osg::ref_ptr<osg::Node> airPlane2 =  osgDB::readNodeFile("./boeing-747.osgb");
//        node->setScale(osg::Vec3d(0.2,0.2,0.2));
//        node->addChild(airPlane);

//        airPlane->setNodeMask(0);

        gp1->addChild(airPlane);
        gp2->addChild(airPlane);


//        gp1->setPosition(osg::Vec3(0,0,20));

//        gp1->setNodeMask(0);
//        gp2->setNodeMask(0);


        root->addChild(gp1);
        root->addChild(gp2);
    viewer.setSceneData(root);
    viewer.realize();




    return viewer.run();
}









