// modificado de https://github.com/pvigier/FortuneAlgorithm

/* FortuneAlgorithm
 * Copyright (C) 2018 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// STL
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <ostream>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// #include "Vector2.h"
// Declarations
class Vector2;
Vector2 operator-(Vector2 lhs, const Vector2& rhs);
// Implementations
class Vector2
{
public:
	double x;
	double y;
	Vector2(double x = 0.0, double y = 0.0);
	// Unary operators
	Vector2 operator-() const;
	Vector2& operator+=(const Vector2& other);
	Vector2& operator-=(const Vector2& other);
	Vector2& operator*=(double t);
	// Other operations
	Vector2 getOrthogonal() const;
	double dot(const Vector2& other) const;
	double getNorm() const;
	double getDistance(const Vector2& other) const;
	double getDet(const Vector2& other) const;
};
// Binary operators
Vector2 operator+(Vector2 lhs, const Vector2& rhs);
Vector2 operator-(Vector2 lhs, const Vector2& rhs);
Vector2 operator*(double t, Vector2 vec);
Vector2 operator*(Vector2 vec, double t);
std::ostream& operator<<(std::ostream& os, const Vector2& vec);

// #include "Box.h"
class Box
{
public:
	// Be careful, y-axis is oriented to the top like in math
	enum class Side : int {LEFT, BOTTOM, RIGHT, TOP};
	struct Intersection
	{
		Side side;
		Vector2 point;
	};
	double left;
	double bottom;
	double right;
	double top;
	bool contains(const Vector2& point) const;
	Intersection getFirstIntersection(const Vector2& origin, const Vector2& direction) const; // Useful for Fortune's algorithm
	int getIntersections(const Vector2& origin, const Vector2& destination, std::array<Intersection, 2>& intersections) const; // Useful for diagram intersection
private:
	static constexpr double EPSILON = std::numeric_limits<double>::epsilon();
};

// #include "VoronoiDiagram.h"
class FortuneAlgorithm; // funciona ??

class VoronoiDiagram
{
public:
    struct HalfEdge;
    struct Face;
    struct Site
    {
        std::size_t index;
        Vector2 point;
        Face* face;
    };
    struct Vertex
    {
        Vector2 point;
    private:
        friend VoronoiDiagram;
        std::list<Vertex>::iterator it;
    };
    struct HalfEdge
    {
        Vertex* origin = nullptr;
        Vertex* destination = nullptr;
        HalfEdge* twin = nullptr;
        Face* incidentFace;
        HalfEdge* prev = nullptr;
        HalfEdge* next = nullptr;
    private:
        friend VoronoiDiagram;
        std::list<HalfEdge>::iterator it;
    };
    struct Face
    {
        Site* site;
        HalfEdge* outerComponent;
    };
    VoronoiDiagram(const std::vector<Vector2>& points);
    // Remove copy operations
    VoronoiDiagram(const VoronoiDiagram&) = delete;
    VoronoiDiagram& operator=(const VoronoiDiagram&) = delete;
    // Move operations
    VoronoiDiagram(VoronoiDiagram&&) = default;
    VoronoiDiagram& operator=(VoronoiDiagram&&) = default;
    // Accessors
    Site* getSite(std::size_t i);
    std::size_t getNbSites() const;
    Face* getFace(std::size_t i);
    const std::list<Vertex>& getVertices() const;
    const std::list<HalfEdge>& getHalfEdges() const;
    // Intersection with a box
    bool intersect(Box box);
private:
    std::vector<Site> mSites;
    std::vector<Face> mFaces;
    std::list<Vertex> mVertices;
    std::list<HalfEdge> mHalfEdges;
    // Diagram construction
    friend FortuneAlgorithm;
    Vertex* createVertex(Vector2 point);
    Vertex* createCorner(Box box, Box::Side side);
    HalfEdge* createHalfEdge(Face* face);
    // Intersection with a box
    void link(Box box, HalfEdge* start, Box::Side startSide, HalfEdge* end, Box::Side endSide);
    void removeVertex(Vertex* vertex);
    void removeHalfEdge(HalfEdge* halfEdge);
};

// #include "Arc.h"
class Event;
struct Arc
{
	enum class Color{RED, BLACK};
	// Hierarchy
	Arc* parent;
	Arc* left;
	Arc* right;
	// Diagram
	VoronoiDiagram::Site* site;
	VoronoiDiagram::HalfEdge* leftHalfEdge;
	VoronoiDiagram::HalfEdge* rightHalfEdge;
	Event* event;
	// Optimizations
	Arc* prev;
	Arc* next;
	// Only for balancing
	Color color;
};

// #include "Event.h"
class Event
{
public:
	enum class Type{SITE, CIRCLE};
	// Site event
	Event(VoronoiDiagram::Site* site);
	// Circle event
	Event(double y, Vector2 point, Arc* arc);

	const Type type;
	double y;
	int index;
	// Site event
	VoronoiDiagram::Site* site;
	// Circle event
	Vector2 point;
	Arc* arc;
};
bool operator<(const Event& lhs, const Event& rhs);
std::ostream& operator<<(std::ostream& os, const Event& event);

// #include "Beachline.h"
class Beachline
{
public:
    Beachline();
    ~Beachline();
    // Remove copy and move operations
    Beachline(const Beachline&) = delete;
    Beachline& operator=(const Beachline&) = delete;
    Beachline(Beachline&&) = delete;
    Beachline& operator=(Beachline&&) = delete;
    Arc* createArc(VoronoiDiagram::Site* site);
    bool isEmpty() const;
    bool isNil(const Arc* x) const;
    void setRoot(Arc* x);
    Arc* getLeftmostArc() const;
    Arc* locateArcAbove(const Vector2& point, double l) const;
    void insertBefore(Arc* x, Arc* y);
    void insertAfter(Arc* x, Arc* y);
    void replace(Arc* x, Arc* y);
    void remove(Arc* z);
    std::ostream& print(std::ostream& os) const;
private:
    Arc* mNil;
    Arc* mRoot;
    // Utility methods
    Arc* minimum(Arc* x) const;
    void transplant(Arc* u, Arc* v); 
    // Fixup functions
    void insertFixup(Arc* z);
    void removeFixup(Arc* x);
    // Rotations
    void leftRotate(Arc* x);
    void rightRotate(Arc* y);

    double computeBreakpoint(const Vector2& point1, const Vector2& point2, double l) const;
    void free(Arc* x);
    std::ostream& printArc(std::ostream& os, const Arc* arc, std::string tabs = "") const;
};
std::ostream& operator<<(std::ostream& os, const Beachline& beachline);

// #include "PriorityQueue.h"
template<typename T>
class PriorityQueue
{
public:
    PriorityQueue()
    {
    }
    // Accessors
    bool isEmpty() const
    {
        return mElements.empty();
    }
    // Operations
    std::unique_ptr<T> pop()
    {
        swap(0, mElements.size() - 1);
        auto top = std::move(mElements.back());
        mElements.pop_back();
        siftDown(0);
        return top;
    }
    void push(std::unique_ptr<T> elem)
    {
        elem->index = mElements.size();
        mElements.emplace_back(std::move(elem));
        siftUp(mElements.size() - 1);
    }
    void update(std::size_t i)
    {
        int parent = getParent(i);
        if(parent >= 0 && *mElements[parent] < *mElements[i])
            siftUp(i);
        else
            siftDown(i);
    }
    void remove(std::size_t i)
    {
        swap(i, mElements.size() - 1);
        mElements.pop_back();
        if (i < mElements.size())
            update(i);
    }
    // Print 
    std::ostream& print(std::ostream& os, int i = 0, std::string tabs = "") const
    {
        if(i < mElements.size())
        {
            os << tabs << *mElements[i] << std::endl;
            display(getLeftChild(i), tabs + '\t');
            display(getRightChild(i), tabs + '\t');
        }
        return os;
    }
private:
    std::vector<std::unique_ptr<T>> mElements;
    // Accessors
    int getParent(int i) const
    {
        return (i + 1) / 2 - 1;
    }
    std::size_t getLeftChild(std::size_t i) const
    {
        return 2 * (i + 1) - 1;
    }
    std::size_t getRightChild(std::size_t i) const
    {
        return 2 * (i + 1);
    }
    // Operations
    void siftDown(std::size_t i)
    {
        std::size_t left = getLeftChild(i);
        std::size_t right = getRightChild(i);
        std::size_t j = i;
        if(left < mElements.size() && *mElements[j] < *mElements[left])
            j = left;
        if(right < mElements.size() && *mElements[j] < *mElements[right])
            j = right;
        if(j != i)
        {
            swap(i, j);
            siftDown(j);
        }
    }
    void siftUp(std::size_t i)
    {
        int parent = getParent(i);
        if(parent >= 0 && *mElements[parent] < *mElements[i])
        {
            swap(i, parent);
            siftUp(parent);
        }
    }
    inline void swap(std::size_t i, std::size_t j)
    {
        auto tmp = std::move(mElements[i]);
        mElements[i] = std::move(mElements[j]);
        mElements[j] = std::move(tmp);
        mElements[i]->index = i;
        mElements[j]->index = j;
    }
};
template <typename T>
std::ostream& operator<<(std::ostream& os, const PriorityQueue<T>& queue)
{
    return queue.print(os);
}

// #include "FortuneAlgorithm.h"
class FortuneAlgorithm
{
public:
    FortuneAlgorithm(std::vector<Vector2> points);
    ~FortuneAlgorithm();
    void construct();
    bool bound(Box box);
    VoronoiDiagram getDiagram();
private:
    VoronoiDiagram mDiagram;
    Beachline mBeachline;
    PriorityQueue<Event> mEvents;
    double mBeachlineY;
    // Algorithm
    void handleSiteEvent(Event* event);
    void handleCircleEvent(Event* event);
    // Arcs
    Arc* breakArc(Arc* arc, VoronoiDiagram::Site* site);
    void removeArc(Arc* arc, VoronoiDiagram::Vertex* vertex);
    // Breakpoint
    bool isMovingRight(const Arc* left, const Arc* right) const;
    double getInitialX(const Arc* left, const Arc* right, bool movingRight) const;
    // Edges
    void addEdge(Arc* left, Arc* right);
    void setOrigin(Arc* left, Arc* right, VoronoiDiagram::Vertex* vertex);
    void setDestination(Arc* left, Arc* right, VoronoiDiagram::Vertex* vertex);
    void setPrevHalfEdge(VoronoiDiagram::HalfEdge* prev, VoronoiDiagram::HalfEdge* next);
    // Events
    void addEvent(Arc* left, Arc* middle, Arc* right);
    void deleteEvent(Arc* arc);
    Vector2 computeConvergencePoint(const Vector2& point1, const Vector2& point2, const Vector2& point3, double& y) const;
    // Bounding
    struct LinkedVertex
    {
        VoronoiDiagram::HalfEdge* prevHalfEdge;
        VoronoiDiagram::Vertex* vertex;
        VoronoiDiagram::HalfEdge* nextHalfEdge;
    };
};

// Vector2.cpp
Vector2::Vector2(double x, double y) : x(x), y(y)
{

}
// Unary operators
Vector2 Vector2::operator-() const
{
    return Vector2(-x, -y);
}
Vector2& Vector2::operator+=(const Vector2& other)
{
    x += other.x;
    y += other.y;
    return *this;
}
Vector2& Vector2::operator-=(const Vector2& other)
{
    x -= other.x;
    y -= other.y;
    return *this;
}
Vector2& Vector2::operator*=(double t)  
{
    x *= t;
    y *= t;
    return *this; 
}
// Other operations
Vector2 Vector2::getOrthogonal() const
{
    return Vector2(-y, x);
}
double Vector2::dot(const Vector2& other) const
{
    return x * other.x + y * other.y;
}
double Vector2::getNorm() const
{
    return std::sqrt(x * x + y * y);
}
double Vector2::getDistance(const Vector2& other) const
{
    return (*this - other).getNorm();
}
double Vector2::getDet(const Vector2& other) const
{
    return x * other.y - y * other.x;
}
Vector2 operator+(Vector2 lhs, const Vector2& rhs)
{
    lhs += rhs;
    return lhs;
}
Vector2 operator-(Vector2 lhs, const Vector2& rhs) 
{
    lhs -= rhs;
    return lhs;
}
Vector2 operator*(double t, Vector2 vec)
{
    vec *= t;
    return vec;
}
Vector2 operator*(Vector2 vec, double t)
{
    return t * vec;
}
std::ostream& operator<<(std::ostream& os, const Vector2& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

// Box.cpp
bool Box::contains(const Vector2& point) const
{
    return point.x >= left - EPSILON && point.x <= right + EPSILON &&
        point.y >= bottom  - EPSILON && point.y <= top + EPSILON;
}

Box::Intersection Box::getFirstIntersection(const Vector2& origin, const Vector2& direction) const
{
    // origin must be in the box
    Intersection intersection;
    double t = std::numeric_limits<double>::infinity();
    if (direction.x > 0.0)
    {
        t = (right - origin.x) / direction.x;
        intersection.side = Side::RIGHT;
        intersection.point = origin + t * direction;
    }
    else if (direction.x < 0.0)
    {
        t = (left - origin.x) / direction.x;
        intersection.side = Side::LEFT;
        intersection.point = origin + t * direction;
    }
    if (direction.y > 0.0)
    {
        double newT = (top - origin.y) / direction.y;
        if (newT < t)
        {
            intersection.side = Side::TOP;
            intersection.point = origin + newT * direction;
        }
    }
    else if (direction.y < 0.0)
    {
        double newT = (bottom - origin.y) / direction.y;
        if (newT < t)
        {
            intersection.side = Side::BOTTOM;
            intersection.point = origin + newT * direction;
        }
    }
    return intersection;
}

int Box::getIntersections(const Vector2& origin, const Vector2& destination, std::array<Intersection, 2>& intersections) const
{
    // WARNING: If the intersection is a corner, both intersections are equals
    Vector2 direction = destination - origin;
    std::array<double, 2> t;
    std::size_t i = 0; // index of the current intersection
    // Left
    if (origin.x < left - EPSILON || destination.x < left - EPSILON)
    {   
        t[i] = (left - origin.x) / direction.x;
        if (t[i] > EPSILON && t[i] < 1.0 - EPSILON)  
        {
            intersections[i].side = Side::LEFT;
            intersections[i].point = origin + t[i] * direction;
            if (intersections[i].point.y >= bottom  - EPSILON && intersections[i].point.y <= top + EPSILON)
                ++i;
        }
    }
    // Right
    if (origin.x > right + EPSILON || destination.x > right + EPSILON)
    {   
        t[i] = (right - origin.x) / direction.x;
        if (t[i] > EPSILON && t[i] < 1.0 - EPSILON)  
        {
            intersections[i].side = Side::RIGHT;
            intersections[i].point = origin + t[i] * direction;
            if (intersections[i].point.y >= bottom - EPSILON && intersections[i].point.y <= top + EPSILON)
                ++i;
        }
    }
    // Bottom
    if (origin.y < bottom - EPSILON || destination.y < bottom - EPSILON)
    {   
        t[i] = (bottom - origin.y) / direction.y;
        if (i < 2 && t[i] > EPSILON && t[i] < 1.0 - EPSILON)  
        {
            intersections[i].side = Side::BOTTOM;
            intersections[i].point = origin + t[i] * direction;
            if (intersections[i].point.x >= left  - EPSILON && intersections[i].point.x <= right + EPSILON)
                ++i;
        }
    }
    // Top
    if (origin.y > top + EPSILON || destination.y > top + EPSILON)
    {   
        t[i] = (top - origin.y) / direction.y;
        if (i < 2 && t[i] > EPSILON && t[i] < 1.0 - EPSILON)  
        {
            intersections[i].side = Side::TOP;
            intersections[i].point = origin + t[i] * direction;
            if (intersections[i].point.x >= left - EPSILON && intersections[i].point.x <= right + EPSILON)
                ++i;
        }
    }
    // Sort the intersections from the nearest to the farthest
    if (i == 2 && t[0] > t[1])
        std::swap(intersections[0], intersections[1]);
    return i;
}

// VoronoiDiagram.cpp
VoronoiDiagram::VoronoiDiagram(const std::vector<Vector2>& points)
{
    mSites.reserve(points.size());
    mFaces.reserve(points.size());
    for(std::size_t i = 0; i < points.size(); ++i)
    {
        mSites.push_back(VoronoiDiagram::Site{i, points[i], nullptr});
        mFaces.push_back(VoronoiDiagram::Face{&mSites.back(), nullptr});
        mSites.back().face = &mFaces.back();
    }
}
VoronoiDiagram::Site* VoronoiDiagram::getSite(std::size_t i)
{
    return &mSites[i];
}
std::size_t VoronoiDiagram::getNbSites() const
{
    return mSites.size();
}
VoronoiDiagram::Face* VoronoiDiagram::getFace(std::size_t i)
{
    return &mFaces[i];
}
const std::list<VoronoiDiagram::Vertex>& VoronoiDiagram::getVertices() const
{
    return mVertices;
}
const std::list<VoronoiDiagram::HalfEdge>& VoronoiDiagram::getHalfEdges() const
{
    return mHalfEdges;
}
bool VoronoiDiagram::intersect(Box box)
{
    bool error = false;
    std::unordered_set<HalfEdge*> processedHalfEdges;
    std::unordered_set<Vertex*> verticesToRemove;
    for (const Site& site : mSites)
    {
        HalfEdge* halfEdge = site.face->outerComponent;
        bool inside = box.contains(halfEdge->origin->point);
        bool outerComponentDirty = !inside;
        HalfEdge* incomingHalfEdge = nullptr; // First half edge coming in the box
        HalfEdge* outgoingHalfEdge = nullptr; // Last half edge going out the box
        Box::Side incomingSide, outgoingSide;
        do
        {
            std::array<Box::Intersection, 2> intersections;
            int nbIntersections = box.getIntersections(halfEdge->origin->point, halfEdge->destination->point, intersections);
            bool nextInside = box.contains(halfEdge->destination->point);
            HalfEdge* nextHalfEdge = halfEdge->next;
            // The two points are outside the box 
            if (!inside && !nextInside)
            {
                // The edge is outside the box
                if (nbIntersections == 0)
                {
                    verticesToRemove.emplace(halfEdge->origin);
                    removeHalfEdge(halfEdge);
                }
                // The edge crosses twice the frontiers of the box
                else if (nbIntersections == 2)
                {
                    verticesToRemove.emplace(halfEdge->origin);
                    if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                    {
                        halfEdge->origin = halfEdge->twin->destination;
                        halfEdge->destination = halfEdge->twin->origin;
                    }
                    else
                    {
                        halfEdge->origin = createVertex(intersections[0].point);
                        halfEdge->destination = createVertex(intersections[1].point);
                    }
                    if (outgoingHalfEdge != nullptr)
                        link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                    if (incomingHalfEdge == nullptr)
                    {
                       incomingHalfEdge = halfEdge;
                       incomingSide = intersections[0].side;
                    }
                    outgoingHalfEdge = halfEdge;
                    outgoingSide = intersections[1].side;
                    processedHalfEdges.emplace(halfEdge);
                }
                else
                    error = true;
            }
            // The edge is going outside the box
            else if (inside && !nextInside)
            {
                if (nbIntersections == 1)
                {
                    if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                        halfEdge->destination = halfEdge->twin->origin;
                    else
                        halfEdge->destination = createVertex(intersections[0].point);
                    outgoingHalfEdge = halfEdge;
                    outgoingSide = intersections[0].side;
                    processedHalfEdges.emplace(halfEdge);
                }
                else
                    error = true;
            }
            // The edge is coming inside the box
            else if (!inside && nextInside)
            {
                if (nbIntersections == 1)
                {
                    verticesToRemove.emplace(halfEdge->origin);
                    if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                        halfEdge->origin = halfEdge->twin->destination;
                    else
                        halfEdge->origin = createVertex(intersections[0].point);
                    if (outgoingHalfEdge != nullptr)
                        link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                    if (incomingHalfEdge == nullptr)
                    {
                       incomingHalfEdge = halfEdge;
                       incomingSide = intersections[0].side;
                    }
                    processedHalfEdges.emplace(halfEdge);
                }
                else
                    error = true;
            }
            halfEdge = nextHalfEdge;
            // Update inside
            inside = nextInside;
        } while (halfEdge != site.face->outerComponent);
        // Link the last and the first half edges inside the box
        if (outerComponentDirty && incomingHalfEdge != nullptr)
            link(box, outgoingHalfEdge, outgoingSide, incomingHalfEdge, incomingSide);
        // Set outer component
        if (outerComponentDirty)
            site.face->outerComponent = incomingHalfEdge;
    }
    // Remove vertices
    for (auto& vertex : verticesToRemove)
        removeVertex(vertex);
    // Return the status
    return !error;
}
VoronoiDiagram::Vertex* VoronoiDiagram::createVertex(Vector2 point)
{
    mVertices.emplace_back();
    mVertices.back().point = point;
    mVertices.back().it = std::prev(mVertices.end());
    return &mVertices.back();
}
VoronoiDiagram::Vertex* VoronoiDiagram::createCorner(Box box, Box::Side side)
{
    switch (side)
    {
        case Box::Side::LEFT:
            return createVertex(Vector2(box.left, box.top));
        case Box::Side::BOTTOM:
            return createVertex(Vector2(box.left, box.bottom));
        case Box::Side::RIGHT:
            return createVertex(Vector2(box.right, box.bottom));
        case Box::Side::TOP:
            return createVertex(Vector2(box.right, box.top));
        default:
            return nullptr;
    }
}
VoronoiDiagram::HalfEdge* VoronoiDiagram::createHalfEdge(Face* face)
{
    mHalfEdges.emplace_back();
    mHalfEdges.back().incidentFace = face;
    mHalfEdges.back().it = std::prev(mHalfEdges.end());
    if(face->outerComponent == nullptr)
        face->outerComponent = &mHalfEdges.back();
    return &mHalfEdges.back();
}
void VoronoiDiagram::link(Box box, HalfEdge* start, Box::Side startSide, HalfEdge* end, Box::Side endSide)
{
    HalfEdge* halfEdge = start;
    int side = static_cast<int>(startSide);
    while (side != static_cast<int>(endSide))
    {
        side = (side + 1) % 4;
        halfEdge->next = createHalfEdge(start->incidentFace);
        halfEdge->next->prev = halfEdge;
        halfEdge->next->origin = halfEdge->destination;
        halfEdge->next->destination = createCorner(box, static_cast<Box::Side>(side));
        halfEdge = halfEdge->next;
    }
    halfEdge->next = createHalfEdge(start->incidentFace);
    halfEdge->next->prev = halfEdge;
    end->prev = halfEdge->next;
    halfEdge->next->next = end;
    halfEdge->next->origin = halfEdge->destination;
    halfEdge->next->destination = end->origin;
}
void VoronoiDiagram::removeVertex(Vertex* vertex)
{
    mVertices.erase(vertex->it);
}
void VoronoiDiagram::removeHalfEdge(HalfEdge* halfEdge)
{
    mHalfEdges.erase(halfEdge->it);
}

// Event.cpp
Event::Event(VoronoiDiagram::Site* site) : type(Type::SITE), y(site->point.y), index(-1), site(site)
{

}

Event::Event(double y, Vector2 point, Arc* arc) : type(Type::CIRCLE), y(y), index(-1), point(point), arc(arc)
{


}
bool operator<(const Event& lhs, const Event& rhs)
{
    return lhs.y < rhs.y;
}

std::ostream& operator<<(std::ostream& os, const Event& event)
{
    if(event.type == Event::Type::SITE)
        os << "S(" << event.site->index << ", " << event.y << ")";
    else
        os << "C(" << event.arc << ", " << event.y << ", " << event.point << ")";
    return os;
}

// Beachline.cpp
Beachline::Beachline() : mNil(new Arc), mRoot(mNil)
{
    mNil->color = Arc::Color::BLACK; 
}

Beachline::~Beachline()
{
    free(mRoot);    
    delete mNil;
}

Arc* Beachline::createArc(VoronoiDiagram::Site* site)
{
    return new Arc{mNil, mNil, mNil, site, nullptr, nullptr, nullptr, mNil, mNil, Arc::Color::RED};
}

bool Beachline::isEmpty() const
{
    return isNil(mRoot);
}

bool Beachline::isNil(const Arc* x) const
{
    return x == mNil;
}

void Beachline::setRoot(Arc* x)
{
    mRoot = x;
    mRoot->color = Arc::Color::BLACK;
}

Arc* Beachline::getLeftmostArc() const
{
    Arc* x = mRoot;
    while (!isNil(x->prev))
        x = x->prev;
    return x;
}

Arc* Beachline::locateArcAbove(const Vector2& point, double l) const
{
    Arc* node = mRoot;
    bool found = false;
    while (!found)
    {
        double breakpointLeft = -std::numeric_limits<double>::infinity();
        double breakpointRight = std::numeric_limits<double>::infinity();
        if (!isNil(node->prev))
           breakpointLeft =  computeBreakpoint(node->prev->site->point, node->site->point, l);
        if (!isNil(node->next))
            breakpointRight = computeBreakpoint(node->site->point, node->next->site->point, l);
        if (point.x < breakpointLeft)
            node = node->left;
        else if (point.x > breakpointRight)
            node = node->right;
        else
            found = true;
    }
    return node;
}

void Beachline::insertBefore(Arc* x, Arc* y)
{
    // Find the right place
    if (isNil(x->left))
    {
        x->left = y;
        y->parent = x;
    }
    else
    {
        x->prev->right = y;
        y->parent = x->prev;
    }
    // Set the pointers
    y->prev = x->prev;
    if (!isNil(y->prev))
        y->prev->next = y;
    y->next = x;
    x->prev = y;
    // Balance the tree
    insertFixup(y);    
}

void Beachline::insertAfter(Arc* x, Arc* y)
{
    // Find the right place
    if (isNil(x->right))
    {
        x->right = y;
        y->parent = x;
    }
    else
    {
        x->next->left = y;
        y->parent = x->next;
    }
    // Set the pointers
    y->next = x->next;
    if (!isNil(y->next))
        y->next->prev = y;
    y->prev = x;
    x->next = y;
    // Balance the tree
    insertFixup(y);    
}

void Beachline::replace(Arc* x, Arc* y)
{
    transplant(x, y);
    y->left = x->left;
    y->right = x->right;
    if (!isNil(y->left))
        y->left->parent = y;
    if (!isNil(y->right))
        y->right->parent = y;
    y->prev = x->prev;
    y->next = x->next;
    if (!isNil(y->prev))
        y->prev->next = y;
    if (!isNil(y->next))
        y->next->prev = y;
    y->color = x->color;
}

void Beachline::remove(Arc* z)
{
    Arc* y = z;
    Arc::Color yOriginalColor = y->color;
    Arc* x;
    if (isNil(z->left))
    {
        x = z->right;
        transplant(z, z->right);
    }
    else if (isNil(z->right))
    {
        x = z->left;
        transplant(z, z->left);
    }
    else
    {
        y = minimum(z->right);
        yOriginalColor = y->color;
        x = y->right;
        if (y->parent == z)
            x->parent = y; // Because x could be Nil
        else
        {
            transplant(y, y->right);
            y->right = z->right;
            y->right->parent = y;
        }
        transplant(z, y);
        y->left = z->left;
        y->left->parent = y;
        y->color = z->color;
    }
    if (yOriginalColor == Arc::Color::BLACK)
        removeFixup(x);
    // Update next and prev
    if (!isNil(z->prev))
        z->prev->next = z->next;
    if (!isNil(z->next))
        z->next->prev = z->prev;
}

std::ostream& Beachline::print(std::ostream& os) const
{
    //return printArc(os, mRoot);
    Arc* arc = getLeftmostArc();
    while (!isNil(arc))
    {
        os << arc->site->index << ' ';
        arc = arc->next;
    }
    return os;
}

Arc* Beachline::minimum(Arc* x) const
{
    while (!isNil(x->left))
        x = x->left;
    return x;
}

void Beachline::transplant(Arc* u, Arc* v)
{
    if (isNil(u->parent))
        mRoot = v;
    else if (u == u->parent->left)
        u->parent->left = v;
    else
        u->parent->right = v;
    v->parent = u->parent;
}

void Beachline::insertFixup(Arc* z)
{
    while (z->parent->color == Arc::Color::RED)
    {
        if (z->parent == z->parent->parent->left)
        {
            Arc* y = z->parent->parent->right;
            // Case 1
            if (y->color == Arc::Color::RED)
            {
                z->parent->color = Arc::Color::BLACK;
                y->color = Arc::Color::BLACK;
                z->parent->parent->color = Arc::Color::RED;
                z = z->parent->parent;
            }
            else
            {
                // Case 2
                if (z == z->parent->right)
                {
                    z = z->parent;
                    leftRotate(z);
                }
                // Case 3
                z->parent->color = Arc::Color::BLACK;
                z->parent->parent->color = Arc::Color::RED;
                rightRotate(z->parent->parent);
            }
        }
        else
        {
            Arc* y = z->parent->parent->left;
            // Case 1
            if (y->color == Arc::Color::RED)
            {
                z->parent->color = Arc::Color::BLACK;
                y->color = Arc::Color::BLACK;
                z->parent->parent->color = Arc::Color::RED;
                z = z->parent->parent;
            }
            else
            {
                // Case 2
                if (z == z->parent->left)
                {
                    z = z->parent;
                    rightRotate(z);
                }
                // Case 3
                z->parent->color = Arc::Color::BLACK;
                z->parent->parent->color = Arc::Color::RED;
                leftRotate(z->parent->parent);
            }
        }
    }
    mRoot->color = Arc::Color::BLACK;
}

void Beachline::removeFixup(Arc* x)
{

    while (x != mRoot && x->color == Arc::Color::BLACK)
    {
        Arc* w;
        if (x == x->parent->left)
        {
            w = x->parent->right;
            // Case 1
            if (w->color == Arc::Color::RED)
            {
                w->color = Arc::Color::BLACK;
                x->parent->color = Arc::Color::RED;
                leftRotate(x->parent);
                w = x->parent->right;
            }
            // Case 2
            if (w->left->color == Arc::Color::BLACK && w->right->color == Arc::Color::BLACK)
            {
                w->color = Arc::Color::RED;
                x = x->parent;
            }
            else
            {
                // Case 3
                if (w->right->color == Arc::Color::BLACK)
                {
                    w->left->color = Arc::Color::BLACK;
                    w->color = Arc::Color::RED;
                    rightRotate(w);
                    w = x->parent->right;
                }
                // Case 4
                w->color = x->parent->color;
                x->parent->color = Arc::Color::BLACK;
                w->right->color = Arc::Color::BLACK;
                leftRotate(x->parent);
                x = mRoot;
            }
        }
        else
        {
            w = x->parent->left;
            // Case 1
            if (w->color == Arc::Color::RED)
            {
                w->color = Arc::Color::BLACK;
                x->parent->color = Arc::Color::RED;
                rightRotate(x->parent);
                w = x->parent->left;
            }
            // Case 2
            if (w->left->color == Arc::Color::BLACK && w->right->color == Arc::Color::BLACK)
            {
                w->color = Arc::Color::RED;
                x = x->parent;
            }
            else
            {
                // Case 3
                if (w->left->color == Arc::Color::BLACK)
                {
                    w->right->color = Arc::Color::BLACK;
                    w->color = Arc::Color::RED;
                    leftRotate(w);
                    w = x->parent->left;
                }
                // Case 4
                w->color = x->parent->color;
                x->parent->color = Arc::Color::BLACK;
                w->left->color = Arc::Color::BLACK;
                rightRotate(x->parent);
                x = mRoot;
            } 
        }
    }
    x->color = Arc::Color::BLACK;
}

void Beachline::leftRotate(Arc* x)
{
    Arc* y = x->right;
    x->right = y->left;
    if (!isNil(y->left))
        y->left->parent = x;
    y->parent = x->parent;
    if (isNil(x->parent))
        mRoot = y;
    else if (x->parent->left == x)
        x->parent->left = y;
    else
        x->parent->right = y;
    y->left = x;
    x->parent = y;
}

void Beachline::rightRotate(Arc* y)
{
    Arc* x = y->left;
    y->left = x->right;
    if (!isNil(x->right))
        x->right->parent = y;
    x->parent = y->parent;
    if (isNil(y->parent))
        mRoot = x;
    else if (y->parent->left == y)
        y->parent->left = x;
    else
        y->parent->right = x;
    x->right = y;
    y->parent = x;
}

double Beachline::computeBreakpoint(const Vector2& point1, const Vector2& point2, double l) const
{
    double x1 = point1.x, y1 = point1.y, x2 = point2.x, y2 = point2.y;
    double d1 = 1.0 / (2.0 * (y1 - l));
	double d2 = 1.0 / (2.0 * (y2 - l));
	double a = d1 - d2;
	double b = 2.0 * (x2 * d2 - x1 * d1);
	double c = (y1 * y1 + x1 * x1 - l * l) * d1 - (y2 * y2 + x2 * x2 - l * l) * d2;
	double delta = b * b - 4.0 * a * c;
    return (-b + std::sqrt(delta)) / (2.0 * a);
}

void Beachline::free(Arc* x)
{
    if (isNil(x))
        return;
    else
    {
        free(x->left);
        free(x->right);
        delete x;
    }
}

std::ostream& Beachline::printArc(std::ostream& os, const Arc* arc, std::string tabs) const
{
    os << tabs << arc->site->index << ' ' << arc->leftHalfEdge << ' ' << arc->rightHalfEdge << std::endl;
    if (!isNil(arc->left))
        printArc(os, arc->left, tabs + '\t');
    if (!isNil(arc->right))
        printArc(os, arc->right, tabs + '\t');
    return os;
}

std::ostream& operator<<(std::ostream& os, const Beachline& beachline)
{
    return beachline.print(os);
}

// FortuneAlgorithm.cpp
FortuneAlgorithm::FortuneAlgorithm(std::vector<Vector2> points) : mDiagram(std::move(points))
{
}
FortuneAlgorithm::~FortuneAlgorithm() = default;
void FortuneAlgorithm::construct()
{
    // Initialize event queue
    for (std::size_t i = 0; i < mDiagram.getNbSites(); ++i)
        mEvents.push(std::make_unique<Event>(mDiagram.getSite(i)));
    // Process events
    while (!mEvents.isEmpty())
    {
        std::unique_ptr<Event> event = mEvents.pop();
        mBeachlineY = event->y;
        if(event->type == Event::Type::SITE)
            handleSiteEvent(event.get());
        else
            handleCircleEvent(event.get());
    }
}
VoronoiDiagram FortuneAlgorithm::getDiagram()
{
    return std::move(mDiagram);
}
void FortuneAlgorithm::handleSiteEvent(Event* event)
{
    VoronoiDiagram::Site* site = event->site;
    // 1. Check if the bachline is empty
    if (mBeachline.isEmpty())
    {
        mBeachline.setRoot(mBeachline.createArc(site));
        return;
    }
    // 2. Look for the arc above the site
    Arc* arcToBreak = mBeachline.locateArcAbove(site->point, mBeachlineY);
    deleteEvent(arcToBreak);
    // 3. Replace this arc by the new arcs
    Arc* middleArc = breakArc(arcToBreak, site);
    Arc* leftArc = middleArc->prev; 
    Arc* rightArc = middleArc->next;
    // 4. Add an edge in the diagram
    addEdge(leftArc, middleArc);
    middleArc->rightHalfEdge = middleArc->leftHalfEdge;
    rightArc->leftHalfEdge = leftArc->rightHalfEdge;
    // 5. Check circle events
    // Left triplet
    if (!mBeachline.isNil(leftArc->prev))
        addEvent(leftArc->prev, leftArc, middleArc);
    // Right triplet
    if (!mBeachline.isNil(rightArc->next))
        addEvent(middleArc, rightArc, rightArc->next);
}
void FortuneAlgorithm::handleCircleEvent(Event* event)
{
    Vector2 point = event->point;
    Arc* arc = event->arc;
    // 1. Add vertex
    VoronoiDiagram::Vertex* vertex = mDiagram.createVertex(point);
    // 2. Delete all the events with this arc
    Arc* leftArc = arc->prev;
    Arc* rightArc = arc->next;
    deleteEvent(leftArc);
    deleteEvent(rightArc);
    // 3. Update the beachline and the diagram
    removeArc(arc, vertex);
    // 4. Add new circle events
    // Left triplet
    if (!mBeachline.isNil(leftArc->prev))
        addEvent(leftArc->prev, leftArc, rightArc);
    // Right triplet
    if (!mBeachline.isNil(rightArc->next))
        addEvent(leftArc, rightArc, rightArc->next);
}
Arc* FortuneAlgorithm::breakArc(Arc* arc, VoronoiDiagram::Site* site)
{
    // Create the new subtree
    Arc* middleArc = mBeachline.createArc(site);
    Arc* leftArc = mBeachline.createArc(arc->site);
    leftArc->leftHalfEdge = arc->leftHalfEdge;
    Arc* rightArc = mBeachline.createArc(arc->site);
    rightArc->rightHalfEdge = arc->rightHalfEdge;
    // Insert the subtree in the beachline
    mBeachline.replace(arc, middleArc);
    mBeachline.insertBefore(middleArc, leftArc);
    mBeachline.insertAfter(middleArc, rightArc);
    // Delete old arc
    delete arc;
    // Return the middle arc
    return middleArc;
}
void FortuneAlgorithm::removeArc(Arc* arc, VoronoiDiagram::Vertex* vertex)
{
    // End edges
    setDestination(arc->prev, arc, vertex);
    setDestination(arc, arc->next, vertex);
    // Join the edges of the middle arc
    arc->leftHalfEdge->next = arc->rightHalfEdge;
    arc->rightHalfEdge->prev = arc->leftHalfEdge;
    // Update beachline
    mBeachline.remove(arc);
    // Create a new edge
    VoronoiDiagram::HalfEdge* prevHalfEdge = arc->prev->rightHalfEdge;
    VoronoiDiagram::HalfEdge* nextHalfEdge = arc->next->leftHalfEdge;
    addEdge(arc->prev, arc->next);
    setOrigin(arc->prev, arc->next, vertex);
    setPrevHalfEdge(arc->prev->rightHalfEdge, prevHalfEdge);
    setPrevHalfEdge(nextHalfEdge, arc->next->leftHalfEdge);
    // Delete node
    delete arc;
}
bool FortuneAlgorithm::isMovingRight(const Arc* left, const Arc* right) const
{
    return left->site->point.y < right->site->point.y;
}
double FortuneAlgorithm::getInitialX(const Arc* left, const Arc* right, bool movingRight) const
{
    return movingRight ? left->site->point.x : right->site->point.x;
}
void FortuneAlgorithm::addEdge(Arc* left, Arc* right)
{
    // Create two new half edges
    left->rightHalfEdge = mDiagram.createHalfEdge(left->site->face);
    right->leftHalfEdge = mDiagram.createHalfEdge(right->site->face);
    // Set the two half edges twins
    left->rightHalfEdge->twin = right->leftHalfEdge;
    right->leftHalfEdge->twin = left->rightHalfEdge;
}
void FortuneAlgorithm::setOrigin(Arc* left, Arc* right, VoronoiDiagram::Vertex* vertex)
{
    left->rightHalfEdge->destination = vertex;
    right->leftHalfEdge->origin = vertex;
}
void FortuneAlgorithm::setDestination(Arc* left, Arc* right, VoronoiDiagram::Vertex* vertex)
{
    left->rightHalfEdge->origin = vertex;
    right->leftHalfEdge->destination = vertex;
}
void FortuneAlgorithm::setPrevHalfEdge(VoronoiDiagram::HalfEdge* prev, VoronoiDiagram::HalfEdge* next)
{
    prev->next = next;
    next->prev = prev;
}
void FortuneAlgorithm::addEvent(Arc* left, Arc* middle, Arc* right)
{
    double y;
    Vector2 convergencePoint = computeConvergencePoint(left->site->point, middle->site->point, right->site->point, y);
    bool isBelow = y <= mBeachlineY;
    bool leftBreakpointMovingRight = isMovingRight(left, middle);
    bool rightBreakpointMovingRight = isMovingRight(middle, right);
    double leftInitialX = getInitialX(left, middle, leftBreakpointMovingRight);
    double rightInitialX = getInitialX(middle, right, rightBreakpointMovingRight);
    bool isValid =
        ((leftBreakpointMovingRight && leftInitialX < convergencePoint.x) ||
        (!leftBreakpointMovingRight && leftInitialX > convergencePoint.x)) &&
        ((rightBreakpointMovingRight && rightInitialX < convergencePoint.x) ||
        (!rightBreakpointMovingRight && rightInitialX > convergencePoint.x));
    if (isValid && isBelow)
    {
        std::unique_ptr<Event> event = std::make_unique<Event>(y, convergencePoint, middle);
        middle->event = event.get();
        mEvents.push(std::move(event));
    }
}
void FortuneAlgorithm::deleteEvent(Arc* arc)
{
    if (arc->event != nullptr)
    {
        mEvents.remove(arc->event->index);
        arc->event = nullptr;
    }
}
Vector2 FortuneAlgorithm::computeConvergencePoint(const Vector2& point1, const Vector2& point2, const Vector2& point3, double& y) const
{
    Vector2 v1 = (point1 - point2).getOrthogonal();
    Vector2 v2 = (point2 - point3).getOrthogonal();
    Vector2 delta = 0.5 * (point3 - point1);
    double t = delta.getDet(v2) / v1.getDet(v2);
    Vector2 center = 0.5 * (point1 + point2) + t * v1;
    double r = center.getDistance(point1);
    y = center.y - r;
    return center;
}
bool FortuneAlgorithm::bound(Box box)
{
    // Make sure the bounding box contains all the vertices
    for (const auto& vertex : mDiagram.getVertices()) // Much faster when using vector<unique_ptr<Vertex*>, maybe we can test vertices in border cells to speed up
    {
        box.left = std::min(vertex.point.x, box.left);
        box.bottom = std::min(vertex.point.y, box.bottom);
        box.right = std::max(vertex.point.x, box.right);
        box.top = std::max(vertex.point.y, box.top);
    }
    // Retrieve all non bounded half edges from the beach line
    std::list<LinkedVertex> linkedVertices;
    std::unordered_map<std::size_t, std::array<LinkedVertex*, 8>> vertices(mDiagram.getNbSites());
    if (!mBeachline.isEmpty())
    {
        Arc* leftArc = mBeachline.getLeftmostArc();
        Arc* rightArc = leftArc->next;
        while (!mBeachline.isNil(rightArc))
        {
            // Bound the edge
            Vector2 direction = (leftArc->site->point - rightArc->site->point).getOrthogonal();
            Vector2 origin = (leftArc->site->point + rightArc->site->point) * 0.5f;
            // Line-box intersection
            Box::Intersection intersection = box.getFirstIntersection(origin, direction);
            // Create a new vertex and ends the half edges
            VoronoiDiagram::Vertex* vertex = mDiagram.createVertex(intersection.point);
            setDestination(leftArc, rightArc, vertex);
            // Initialize pointers
            if (vertices.find(leftArc->site->index) == vertices.end()) 
                vertices[leftArc->site->index].fill(nullptr); 
            if (vertices.find(rightArc->site->index) == vertices.end()) 
                vertices[rightArc->site->index].fill(nullptr); 
            // Store the vertex on the boundaries
            linkedVertices.emplace_back(LinkedVertex{nullptr, vertex, leftArc->rightHalfEdge});
            vertices[leftArc->site->index][2 * static_cast<int>(intersection.side) + 1] = &linkedVertices.back();
            linkedVertices.emplace_back(LinkedVertex{rightArc->leftHalfEdge, vertex, nullptr});
            vertices[rightArc->site->index][2 * static_cast<int>(intersection.side)] = &linkedVertices.back();
            // Next edge
            leftArc = rightArc;
            rightArc = rightArc->next;
        }
    }
    // Add corners
    for (auto& kv : vertices)
    {
        auto& cellVertices = kv.second;
        // We check twice the first side to be sure that all necessary corners are added
        for (std::size_t i = 0; i < 5; ++i)
        {
            std::size_t side = i % 4;
            std::size_t nextSide = (side + 1) % 4;
            // Add first corner
            if (cellVertices[2 * side] == nullptr && cellVertices[2 * side + 1] != nullptr)
            {
                std::size_t prevSide = (side + 3) % 4;
                VoronoiDiagram::Vertex* corner = mDiagram.createCorner(box, static_cast<Box::Side>(side));
                linkedVertices.emplace_back(LinkedVertex{nullptr, corner, nullptr});
                cellVertices[2 * prevSide + 1] = &linkedVertices.back();
                cellVertices[2 * side] = &linkedVertices.back();
            }
            // Add second corner
            else if (cellVertices[2 * side] != nullptr && cellVertices[2 * side + 1] == nullptr)
            {
                VoronoiDiagram::Vertex* corner = mDiagram.createCorner(box, static_cast<Box::Side>(nextSide));
                linkedVertices.emplace_back(LinkedVertex{nullptr, corner, nullptr});
                cellVertices[2 * side + 1] = &linkedVertices.back();
                cellVertices[2 * nextSide] = &linkedVertices.back();
            }
        }
    }
    // Join the half edges
    for (auto& kv : vertices)
    {
        std::size_t i = kv.first;
        auto& cellVertices = kv.second;
        for (std::size_t side = 0; side < 4; ++side)
        {
            if (cellVertices[2 * side] != nullptr)
            {
                // Link vertices 
                VoronoiDiagram::HalfEdge* halfEdge = mDiagram.createHalfEdge(mDiagram.getFace(i));
                halfEdge->origin = cellVertices[2 * side]->vertex;
                halfEdge->destination = cellVertices[2 * side + 1]->vertex;
                cellVertices[2 * side]->nextHalfEdge = halfEdge;
                halfEdge->prev = cellVertices[2 * side]->prevHalfEdge;
                if (cellVertices[2 * side]->prevHalfEdge != nullptr)
                    cellVertices[2 * side]->prevHalfEdge->next = halfEdge;
                cellVertices[2 * side + 1]->prevHalfEdge = halfEdge;
                halfEdge->next = cellVertices[2 * side + 1]->nextHalfEdge;
                if (cellVertices[2 * side + 1]->nextHalfEdge != nullptr)
                    cellVertices[2 * side + 1]->nextHalfEdge->prev = halfEdge;
            }
        }
    }
    return true; // TO DO: detect errors
}
