# osgCookbook2.6
Exercício 2.6 "Designing a breadth-first node visitor"

Para compilar:

`g++ cook2.6.cpp -losg -losgAnimation -losgDB -losgUtil -losgViewer -o cook2.6`

Saída:

```
DFS Visitor traversal: 
osg::Group
  osg::MatrixTransform
    osgParticle::ModularEmitter
    osgParticle::ModularEmitter
    osgParticle::ModularEmitter
  osgParticle::ParticleSystemUpdater
  osg::Geode
    osgParticle::ParticleSystem

BFS Visitor traversal: 
osg::Group
osg::MatrixTransform
osgParticle::ParticleSystemUpdater
osg::Geode
osgParticle::ModularEmitter
osgParticle::ModularEmitter
osgParticle::ModularEmitter
osgParticle::ParticleSystem
```
