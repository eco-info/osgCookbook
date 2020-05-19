// Designing a breadth-first node visitor

#include <osg/NodeVisitor>
#include <deque>

#include <osgDB/ReadFile>
#include <osgUtil/PrintVisitor>
#include <iostream>

class BFSVisitor : public osg::NodeVisitor
{
public:
	//BFSVisitor() { setVisitorType(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN); }
	BFSVisitor() { setVisitorType(osg::NodeVisitor::NODE_VISITOR); }
	virtual void reset() { _pendingNodes.clear(); }
	virtual void apply( osg::Node& node ) { traverseBFS(node); }
protected:
	virtual ~BFSVisitor() {}
	void traverseBFS( osg::Node& node );
	std::deque<osg::Node*> _pendingNodes;
};

void BFSVisitor::traverseBFS( osg::Node& node )
{
	osg::Group* group = node.asGroup();
	if ( !group )
		return;
	for ( unsigned int i=0; i<group->getNumChildren(); ++i )
	{
		_pendingNodes.push_back( group->getChild(i) );
	}
	while ( _pendingNodes.size()>0 )
	{
		osg::Node* node = _pendingNodes.front();
		_pendingNodes.pop_front();
		node->accept(*this);
	}
}

class BFSPrintVisitor : public BFSVisitor
{
public:
	virtual void apply( osg::Node& node )
	{
		std::cout << node.libraryName() << "::" << node.className() << std::endl;
		traverseBFS(node);
	}
};

int main( int argc, char** argv )
{
	osg::ArgumentParser arguments( &argc, argv );
	osg::ref_ptr<osg::Node> root = osgDB::readNodeFiles( arguments );
	if ( !root ) root = osgDB::readNodeFile("osgcool.osgt");

	std::cout << "DFS Visitor traversal: " << std::endl;
	osgUtil::PrintVisitor pv( std::cout );
	root->accept( pv );
	std::cout << std::endl;

	std::cout << "BFS Visitor traversal: " << std::endl;
	BFSPrintVisitor bpv;
	root->accept( bpv );
	return 0;
}
