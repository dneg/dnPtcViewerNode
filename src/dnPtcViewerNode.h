#ifndef __DNPTCVIEWERNODE__
#define __DNPTCVIEWERNODE__

#include <maya/MPxLocatorNode.h>

#include <maya/MVectorArray.h>
#include <maya/MFloatArray.h>
#include <maya/MBoundingBox.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MTime.h>

#include <GL/glu.h>


#define USE_CALLBACKS 0
#define USE_CONVERTER 0


class dnPtcViewerNode : public MPxLocatorNode
{
public:
    static MTypeId id;

    static MObject aFilterChange;
    static MObject aDummy;
    static MObject aPtcFile;
    static MObject aPercent;
    static MObject aChannels;
    static MObject aViewAs;
    static MObject aExposure;
    static MObject aPointSize;
    static MObject aSmooth;
    static MObject aInvert;
    static MObject aAbsoluteValue;
    static MObject aShowNormals;
    static MObject aNormalsSize;
    static MObject aInvertNormalsColor;
    static MObject aChannelNames;
    static MObject aNumPoints;
    static MObject aNumPointsLoaded;
    static MObject aUseCropBox;
    static MObject aCropBoxMin;
    static MObject aCropBoxMax;
    static MObject aTimeSync;
    static MObject aTime;
    static MObject aShowValue;
    static MObject aFilterVariable;
    static MObject aFilterMode;
    static MObject aFilterValf1;
    static MObject aFilterValf2;
    static MObject aFilterValc1;
    static MObject aFilterValc2;
    static MObject aCircleSlices;
    static MObject aDiskSlices;
    static MObject aFilterOn;

    dnPtcViewerNode();
    virtual ~dnPtcViewerNode();

    virtual void postConstructor();

    virtual MStatus compute( const MPlug& plug, MDataBlock& data );

    virtual void draw( M3dView & view,
                    const MDagPath & path,
                    M3dView::DisplayStyle style,
                    M3dView::DisplayStatus displaystatus
                    );

    virtual bool isBounded() const;
    virtual MBoundingBox boundingBox() const;

    static  void* creator();
    static  MStatus initialize();
    MStatus loadPtc();
    double  lerp( double a, double b, double v ) { return (a*(1.0-v)+b*v); };
    void draw_bbox( bool state );
    void draw_cropBox( bool state );
    void compute_cropBox_world();
    //void compute_cropBox_normalized();
    inline bool isInCropBox( float *point );
    void resetDisplayList( int &list );
    void forceUIRefresh();
    MStatus connectDummyToShear();
    MStatus connectToTime();
    MString getPtcFrame();
    inline bool filterPoint( MVector &val, float &radius, bool load );
    double fclamp( double x,double a,double b) const {return fmax(a,fmin(x,b));};


private:
    bool           m_bDummyConnected;
    bool           m_bFilterChangeConnected;
    bool           m_timeConnected;
    MBoundingBox   m_bbox;
    GLUquadricObj *m_quadricObj;
    MString        m_ptcFile;
    MVectorArray   m_pts;
    MVectorArray   m_ptsColor;
    MVectorArray   m_ptsNormal;
    MFloatArray    m_ptsRadius;
    MString        m_channelType;
    int            m_chan;
    int            m_viewAs;
    int            m_nPoints;
    int            m_nPointsLoaded;
    float          m_percent;
    float          m_exp;
    float          m_ptSize;
    bool           m_smooth;
    bool           m_invert;
    bool           m_abs;
    bool           m_normals;
    float          m_normalsSize;
    bool           m_invertNormalsColor;
    MStringArray   m_channelNames;
    bool           m_doCropBox;
    MBoundingBox   m_cropBox;
    MBoundingBox   m_cropBoxLocal;
    bool           m_timeSync;
    MTime          m_time;
    bool           m_showValue;
    int            m_filterVariable;
    int            m_filterMode;
    float          m_filterValf1;
    float          m_filterValf2;
    MVector        m_filterValc1;
    MVector        m_filterValc2;
    int            m_circleSlices;
    int            m_diskSlices;
    int            m_filterOn;
    // display lists IDs
    int            m_bboxID;
    int            m_cropboxID;
    int            m_ptcID;
    int            m_diskID;

};





#endif // __DNPTCVIEWERNODE__
