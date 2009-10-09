#include "dnPtcViewerNode.h"

#include <maya/MFnPlugin.h>
#include <maya/MFnStringArrayData.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MSelectionList.h>
#include <maya/MDGModifier.h>
#include <maya/MFnDagNode.h>
#include <maya/MDagPath.h>
#include <maya/MAnimControl.h>
#include <maya/MFileObject.h>
#include <maya/MGlobal.h>
#include <maya/M3dView.h>
#include <maya/MProgressWindow.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <boost/shared_array.hpp>

#include <pointcloud.h>

#define dnPtcViewerNodeId 0x889002


#define VALUE_FILTER  0
#define RADIUS_FILTER 1
#define FILTER_PASSTHROUGH	0
#define FILTER_LESSTHAN		1
#define FILTER_MORETHAN		2
#define FILTER_EQUAL		3
#define FILTER_BETWEEN		4
#define FILTER_ON_LOAD		0
#define FILTER_ON_DRAW		1
#define RAD_TO_DEG 57.29577951308232087679
#define DEG_TO_RAD  .01745329251994329576
#define CHECKERR(STAT,MSG)                                 \
  if ( MS::kSuccess != STAT ) {                            \
	cerr <<"  [ Failed ] : " <<MSG << " : "<<STAT<< endl;  \
	return MS::kFailure;                                   \
  }


#ifdef _DEBUG
#   define DEBUG(msg)    cout <<"   [ dnPtcViewerNode ]  "<<msg<<endl<<flush
#   define DEBUGV(msg,x) cout <<"   [ dnPtcViewerNode ]  "<<msg<<x<<endl<<flush
#else
#   define DEBUG(msg)
#   define DEBUGV(msg,x)
#endif

#define ERR(msg)                                        \
cerr <<"   [ dnPtcViewerNode ]  ERROR : "<<msg<<endl;   \
MGlobal::displayError( msg );

#define ERRV(msg,x)                                        \
cerr <<"   [ dnPtcViewerNode ]  ERROR : "<<msg<<x<<endl;   \
MGlobal::displayError( msg );


MTypeId dnPtcViewerNode::id( dnPtcViewerNodeId );

MObject dnPtcViewerNode::aFilterChange;
MObject dnPtcViewerNode::aDummy;
MObject dnPtcViewerNode::aPtcFile;
MObject dnPtcViewerNode::aChannels;
MObject dnPtcViewerNode::aViewAs;
MObject dnPtcViewerNode::aPercent;
MObject dnPtcViewerNode::aExposure;
MObject dnPtcViewerNode::aPointSize;
MObject dnPtcViewerNode::aSmooth;
MObject dnPtcViewerNode::aInvert;
MObject dnPtcViewerNode::aAbsoluteValue;
MObject dnPtcViewerNode::aShowNormals;
MObject dnPtcViewerNode::aNormalsSize;
MObject dnPtcViewerNode::aInvertNormalsColor;
MObject dnPtcViewerNode::aChannelNames;
MObject dnPtcViewerNode::aNumPoints;
MObject dnPtcViewerNode::aNumPointsLoaded;
MObject dnPtcViewerNode::aUseCropBox;
MObject dnPtcViewerNode::aCropBoxMin;
MObject dnPtcViewerNode::aCropBoxMax;
MObject dnPtcViewerNode::aTimeSync;
MObject dnPtcViewerNode::aTime;
MObject dnPtcViewerNode::aShowValue;
MObject dnPtcViewerNode::aFilterVariable;
MObject dnPtcViewerNode::aFilterMode;
MObject dnPtcViewerNode::aFilterValf1;
MObject dnPtcViewerNode::aFilterValf2;
MObject dnPtcViewerNode::aFilterValc1;
MObject dnPtcViewerNode::aFilterValc2;
MObject dnPtcViewerNode::aCircleSlices;
MObject dnPtcViewerNode::aDiskSlices;
MObject dnPtcViewerNode::aFilterOn;


//
//  dnPtcViewerNode::Constructor
//
dnPtcViewerNode::dnPtcViewerNode()
{
	m_pts.clear();
	m_ptsColor.clear();
	m_ptsNormal.clear();
	m_ptsRadius.clear();
	m_channelNames.clear();
	m_bDummyConnected     = false;
	m_bFilterChangeConnected = false;
	m_timeConnected       = false;
	m_quadricObj          = gluNewQuadric();
	m_ptcFile             = "";
	m_nPoints             = 0;
	m_nPointsLoaded       = 0;
	m_percent             = 100.0;
	m_chan                = 0;
	m_viewAs              = 0;
	m_channelType         = "";
	m_exp                 = 0.0;
	m_ptSize              = 0.0;
	m_smooth              = false;
	m_invert              = false;
	m_abs                 = false;
	m_normals             = false;
	m_normalsSize         = 1.0;
	m_invertNormalsColor  = false;
	m_bbox                = MBoundingBox( MPoint( -1.0, -1.0, -1.0 ), MPoint( 1.0, 1.0, 1.0 ) );
	m_doCropBox           = false;
	m_cropBox             = MBoundingBox( MPoint(0.0,0.0,0.0), MPoint(1.0,1.0,1.0) );
	m_cropBoxLocal        = MBoundingBox( MPoint(0.0,0.0,0.0), MPoint(1.0,1.0,1.0) );
	m_timeSync            = false;
	m_time.setValue(0.0);
	m_showValue           = false;
	m_circleSlices        = 8;
	m_diskSlices          = 8;
	m_filterOn            = 0;
	m_filterVariable      = 1;
	m_filterMode          = 0;
	m_filterValf1         = 0.0;
	m_filterValf2         = 0.0;
	m_filterValc1         = MVector(0.0,0.0,0.0);
	m_filterValc2         = MVector(1.0,1.0,1.0);;

	m_bboxID    = -1;
	m_cropboxID = -1;
	m_ptcID     = -1;
	m_diskID    = -1;
}


//
//  dnPtcViewerNode::Destructor
//
dnPtcViewerNode::~dnPtcViewerNode()
{
	resetDisplayList( m_bboxID );
	resetDisplayList( m_cropboxID );
	resetDisplayList( m_ptcID );
	resetDisplayList( m_diskID );

	if(m_quadricObj == NULL) gluDeleteQuadric(m_quadricObj);
}


void dnPtcViewerNode::postConstructor()
{
  MFnDependencyNode nodeFn(thisMObject());
  nodeFn.setName("ptcViewerShape#");
}


//
//  dnPtcViewerNode::compute_cropBox_world
//
void dnPtcViewerNode::compute_cropBox_world()
{
	m_cropBoxLocal.clear();
	m_cropBoxLocal = MBoundingBox(
						MPoint( fclamp( lerp( m_bbox.min().x, m_bbox.max().x, (double)m_cropBox.min().x ), m_bbox.min().x, m_bbox.max().x),
								fclamp( lerp( m_bbox.min().y, m_bbox.max().y, (double)m_cropBox.min().y ), m_bbox.min().y, m_bbox.max().y),
								fclamp( lerp( m_bbox.min().z, m_bbox.max().z, (double)m_cropBox.min().z ), m_bbox.min().z, m_bbox.max().z) ),
						MPoint( fclamp( lerp( m_bbox.min().x, m_bbox.max().x, (double)m_cropBox.max().x ), m_bbox.min().x, m_bbox.max().x),
								fclamp( lerp( m_bbox.min().y, m_bbox.max().y, (double)m_cropBox.max().y ), m_bbox.min().y, m_bbox.max().y),
								fclamp( lerp( m_bbox.min().z, m_bbox.max().z, (double)m_cropBox.max().z ), m_bbox.min().z, m_bbox.max().z) )
								);

}

//void dnPtcViewerNode::compute_cropBox_normalized()
//{
//	m_cropBoxMin = MVector( fclamp(((double)m_cropBoxMinLocal.x - m_bbox.min().x)/(m_bbox.max().x - m_bbox.min().x), 0.0, 1.0),
//							fclamp(((double)m_cropBoxMinLocal.y - m_bbox.min().y)/(m_bbox.max().y - m_bbox.min().y), 0.0, 1.0),
//							fclamp(((double)m_cropBoxMinLocal.z - m_bbox.min().z)/(m_bbox.max().z - m_bbox.min().z), 0.0, 1.0));
//	m_cropBoxMax = MVector( fclamp(((double)m_cropBoxMaxLocal.x - m_bbox.min().x)/(m_bbox.max().x - m_bbox.min().x), 0.0, 1.0),
//							fclamp(((double)m_cropBoxMaxLocal.y - m_bbox.min().y)/(m_bbox.max().y - m_bbox.min().y), 0.0, 1.0),
//							fclamp(((double)m_cropBoxMaxLocal.z - m_bbox.min().z)/(m_bbox.max().z - m_bbox.min().z), 0.0, 1.0));
//}



//
//  dnPtcViewerNode::isInCropBox
//
inline bool dnPtcViewerNode::isInCropBox( float *point )
{
	if (    point[0] > m_cropBoxLocal.min().x && point[0] < m_cropBoxLocal.max().x &&
			point[1] > m_cropBoxLocal.min().y && point[1] < m_cropBoxLocal.max().y &&
			point[2] > m_cropBoxLocal.min().z && point[2] < m_cropBoxLocal.max().z
	) return true;
	return false;
}


//
//  dnPtcViewerNode::resetDisplayList
//
void dnPtcViewerNode::resetDisplayList( int &list )
{
	if ( list == -1 ) return;
	glDeleteLists( list, 1 );
	list = -1;
	DEBUG(" - Reset list");
}


//
//  dnPtcViewerNode::forceUIRefresh
//
void dnPtcViewerNode::forceUIRefresh()
{
	if ( MGlobal::mayaState() == MGlobal::kInteractive ) {
		DEBUG( "REFRESH AE" );
		MGlobal::executeCommandOnIdle( "dnPtcForceAEUpdate();", false );
	}
}


//
//  dnPtcViewerNode::compute
//
MStatus dnPtcViewerNode::compute( const MPlug& plug, MDataBlock& data )
{
	bool reload   = false;
	bool refilter = false;
	bool redraw   = false;
	MStatus returnStatus = MS::kUnknownParameter;
	MStatus status;

	DEBUG("COMPUTE --------------------------------------------");

	if ( plug == aFilterChange )
	{
		DEBUG(" + filter change");

		int old_filterOn = m_filterOn;

		m_filterVariable      = data.inputValue( aFilterVariable ).asShort();
		m_filterMode          = data.inputValue( aFilterMode ).asShort();
		m_filterValf1         = data.inputValue( aFilterValf1 ).asFloat();
		m_filterValf2         = data.inputValue( aFilterValf2 ).asFloat();
		m_filterValc1         = data.inputValue( aFilterValc1 ).asFloatVector();
		m_filterValc2         = data.inputValue( aFilterValc2 ).asFloatVector();
		m_filterOn            = data.inputValue( aFilterOn ).asShort();

		DEBUGV("   + m_filterOn = ",m_filterOn);
		DEBUGV("   + filter color 1: ",m_filterValc1);

		// update to do list
		//
		refilter = true;
		if ( m_filterOn == FILTER_ON_LOAD ) reload = true;
		if ( old_filterOn != m_filterOn ) reload = true;

		returnStatus = MS::kSuccess;
	}

	if ( plug == aDummy )
	{
		DEBUG(" + param change");

		MString old_ptcFile       = m_ptcFile;
		int     old_channel       = m_chan;
		bool    old_doCropBox     = m_doCropBox;
		float   old_percent       = m_percent;
		MTime   old_time          = m_time;
		bool    old_timeSync      = m_timeSync;
		MBoundingBox old_cropBox  = m_cropBox;

		// get input values
		//
		m_ptcFile             = data.inputValue( aPtcFile ).asString();
		m_exp                 = powf( 2.0f, data.inputValue( aExposure ).asFloat() );
		m_ptSize              = data.inputValue( aPointSize ).asFloat();
		m_smooth              = data.inputValue( aSmooth ).asBool();
		m_percent             = data.inputValue( aPercent ).asFloat();
		m_chan                = data.inputValue( aChannels ).asInt();
		m_viewAs              = data.inputValue( aViewAs ).asInt();
		m_invert              = data.inputValue( aInvert ).asBool();
		m_abs                 = data.inputValue( aAbsoluteValue ).asBool();
		m_normals             = data.inputValue( aShowNormals ).asBool();
		m_normalsSize         = data.inputValue( aNormalsSize ).asFloat();
		m_invertNormalsColor  = data.inputValue( aInvertNormalsColor ).asBool();

		// Cosmetic adjustment
		// Is smoot points are on, increase the point size by one
		// to keep the perceived point size constant.
		//
		if ( m_smooth ) m_ptSize += 1;

		// get the crop box values
		// we will limit the values to make sure it
		// remains inside the main bbox and that the crop box
		// doesn't fold over itself.
		//
		MPoint minP           = data.inputValue( aCropBoxMin ).asFloatVector();
		MPoint maxP           = data.inputValue( aCropBoxMax ).asFloatVector();
		minP.x = fmin(minP.x, m_cropBox.max().x);
		minP.y = fmin(minP.y, m_cropBox.max().y);
		minP.z = fmin(minP.z, m_cropBox.max().z);
		maxP.x = fmax(maxP.x, m_cropBox.min().x);
		maxP.y = fmax(maxP.y, m_cropBox.min().y);
		maxP.z = fmax(maxP.z, m_cropBox.min().z);
		// make sure the attributes are appropriately limited
		if ( minP != m_cropBox.min() )
		{
			MFnNumericAttribute minAttrFn(aCropBoxMin);
			minAttrFn.setMax( fmin((double)maxP.x, m_bbox.max().x),
							  fmin((double)maxP.y, m_bbox.max().y),
							  fmin((double)maxP.z, m_bbox.max().z) );
		} else if ( maxP != m_cropBox.max() )
		{
			MFnNumericAttribute maxAttrFn(aCropBoxMax);
			maxAttrFn.setMin( fmin((double)minP.x, m_bbox.min().x),
							  fmin((double)minP.y, m_bbox.min().y),
							  fmin((double)minP.z, m_bbox.min().z) );
		}
		m_cropBox             = MBoundingBox(minP,maxP);
		m_doCropBox           = data.inputValue( aUseCropBox ).asBool();

		// the rest of the inputs
		//
		m_timeSync            = data.inputValue( aTimeSync ).asBool();
		m_time                = data.inputValue( aTime ).asTime();
		m_showValue           = data.inputValue( aShowValue ).asBool();
		m_circleSlices        = data.inputValue( aCircleSlices ).asInt();
		m_diskSlices          = data.inputValue( aDiskSlices ).asInt();


		//
		// update to do list
		//
		redraw = true;

		if ( reload == true ||
			 old_ptcFile   !=  m_ptcFile  ||
			 old_channel   !=  m_chan     ||
			 old_percent   !=  m_percent  ||
			 old_timeSync  !=  m_timeSync ||
			 ( m_timeSync && old_time != m_time ) ||
			 old_doCropBox !=  m_doCropBox ||
			 ( m_doCropBox && ( old_cropBox.min() != m_cropBox.min() || old_cropBox.max() != m_cropBox.max() ) )
		   ) reload = true;

		returnStatus = MS::kSuccess;
	}

	DEBUGV("   + reload = ",reload);

	if ( reload )
	{
		if ( m_ptcFile != "" )
		{

			MFileObject fileObj;
			fileObj.setRawFullName(m_ptcFile);

			if ( ! fileObj.exists() )
			{

				// file does not exist
				m_ptcFile = "";
				m_pts.clear();
				m_ptsColor.clear();
				m_ptsNormal.clear();
				m_ptsRadius.clear();
				ERR("point cloud does not exist");

			}
			else
			{
				DEBUG(" + reload ptc");

				status = loadPtc();
				resetDisplayList( m_bboxID );
				forceUIRefresh();

				if ( status != MS::kSuccess )
				{
					return MS::kFailure;
				}

				//
				// fill output attributes
				//

				// set the name of the channels
				//
				MDataHandle namesHdl = data.outputValue( aChannelNames, &status );
				if ( status != MS::kSuccess )
				{
					ERR("getting aChannelNames array handle");
					return MS::kFailure;
				}
				MFnStringArrayData strData;
				MObject strObj = strData.create( m_channelNames );
				status = namesHdl.set( strObj );
				if ( status != MS::kSuccess )
				{
					ERR("writing channelNames to attr");
					return MS::kFailure;
				}
				namesHdl.setClean();

				// set the number of points
				//
				MDataHandle numHdl = data.outputValue( aNumPoints, &status );
				numHdl.set( m_nPoints );
				numHdl.setClean();

				// set the number of points loaded
				//
				numHdl = data.outputValue( aNumPointsLoaded, &status );
				numHdl.set( m_nPointsLoaded );
				numHdl.setClean();
			}
		}
	}

	// safer to recompute every time.
	//
	compute_cropBox_world();
	resetDisplayList( m_bboxID );
	resetDisplayList( m_cropboxID );
	resetDisplayList( m_ptcID );
	resetDisplayList( m_diskID );

	return returnStatus;
}


//
//  dnPtcViewerNode::draw_bbox
//
void dnPtcViewerNode::draw_bbox( bool state )
{
	if ( m_bboxID < 0 )
	{

		m_bboxID = glGenLists(1);
		glNewList( m_bboxID, GL_COMPILE_AND_EXECUTE );

			glPushAttrib( GL_ALL_ATTRIB_BITS);

			if ( state == true )
			{
				short pattern = 37449;
				glLineStipple( 1, pattern );
				glEnable( GL_LINE_STIPPLE );
			}

			if ( m_smooth )
			{
				glEnable(GL_BLEND);
				glEnable(GL_LINE_SMOOTH);
				glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
			}

			glBegin( GL_LINES );

				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.max().z );
				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.max().z );

				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.min().z );
				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.min().z );

				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.max().z );
				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.max().z );

				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.min().z );
				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.min().z );

				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.min().z );
				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.max().z );

				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.min().z );
				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.max().z );

				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.min().z );
				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.max().z );

				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.min().z );
				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.max().z );

				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.min().z );
				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.min().z );

				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.min().z );
				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.min().z );

				glVertex3f( m_bbox.max().x, m_bbox.min().y, m_bbox.max().z );
				glVertex3f( m_bbox.max().x, m_bbox.max().y, m_bbox.max().z );

				glVertex3f( m_bbox.min().x, m_bbox.min().y, m_bbox.max().z );
				glVertex3f( m_bbox.min().x, m_bbox.max().y, m_bbox.max().z );

			glEnd();

			// center coord sys
			float length = fmin(fmin( m_bbox.width(), m_bbox.height() ), m_bbox.depth() ) * 0.1;
			glDisable( GL_LINE_STIPPLE );
			glBegin( GL_LINES );

				glColor3f( 1.0, 0.0, 0.0 );
				glVertex3f( m_bbox.center().x, m_bbox.center().y, m_bbox.center().z );
				glVertex3f( m_bbox.center().x+length, m_bbox.center().y, m_bbox.center().z );

				glColor3f( 0.0, 1.0, 0.0 );
				glVertex3f( m_bbox.center().x, m_bbox.center().y, m_bbox.center().z );
				glVertex3f( m_bbox.center().x, m_bbox.center().y+length, m_bbox.center().z );

				glColor3f( 0.0, 0.0, 1.0 );
				glVertex3f( m_bbox.center().x, m_bbox.center().y, m_bbox.center().z );
				glVertex3f( m_bbox.center().x, m_bbox.center().y, m_bbox.center().z+length );

			glEnd();

			glPopAttrib();

		glEndList();

	}
	else
	{
		glCallList( m_bboxID );
	}

}


//
//  dnPtcViewerNode::draw_cropBox
//
void dnPtcViewerNode::draw_cropBox( bool state )
{
	if ( m_cropboxID < 0 )
	{

		m_cropboxID = glGenLists(1);
		glNewList( m_cropboxID, GL_COMPILE_AND_EXECUTE );

			glPushAttrib( GL_ALL_ATTRIB_BITS);

				if ( state == true )
				{
					short pattern = 37449;
					glLineStipple( 1, pattern );
					glEnable( GL_LINE_STIPPLE );
				}

				if ( m_smooth )
				{
					glEnable(GL_BLEND);
					glEnable(GL_LINE_SMOOTH);
					glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
				}

				glBegin( GL_LINES );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );

					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.min().z );

					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

				glEnd();

				glDisable( GL_LINE_STIPPLE );
				glLineWidth(3.0);

				float w = m_cropBoxLocal.width()  * 0.2;
				float h = m_cropBoxLocal.height() * 0.2;
				float d = m_cropBoxLocal.depth()  * 0.2;

				// min coord sys
				glBegin( GL_LINES );
					glColor3f( 1.0, 0.0, 0.0 );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x+w, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );

					glColor3f( 0.0, 1.0, 0.0 );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y+h, m_cropBoxLocal.min().z );

					glColor3f( 0.0, 0.0, 1.0 );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z );
					glVertex3f( m_cropBoxLocal.min().x, m_cropBoxLocal.min().y, m_cropBoxLocal.min().z+d );
				glEnd();

				// max coord sys
				glBegin( GL_LINES );
					glColor3f( 1.0, 0.0, 0.0 );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x-w, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );

					glColor3f( 0.0, 1.0, 0.0 );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y-h, m_cropBoxLocal.max().z );

					glColor3f( 0.0, 0.0, 1.0 );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z );
					glVertex3f( m_cropBoxLocal.max().x, m_cropBoxLocal.max().y, m_cropBoxLocal.max().z-d );
				glEnd();

			glPopAttrib();
		glEndList();

	}
	else
	{
		glCallList( m_cropboxID );
	}

}


inline bool dnPtcViewerNode::filterPoint( MVector &val, float &radius, bool load )
{

	if ( m_filterMode == FILTER_PASSTHROUGH ||
		( load && m_filterOn == FILTER_ON_DRAW )
		) return true;

	switch( m_filterVariable )
	{
		case VALUE_FILTER:
		{
			switch( m_filterMode )
			{
				case FILTER_LESSTHAN:
				{
					return ( val.x < m_filterValc1.x &&
							 val.y < m_filterValc1.y &&
							 val.z < m_filterValc1.z );
				}
				case FILTER_MORETHAN:
				{
					return ( val.x > m_filterValc1.x &&
							 val.y > m_filterValc1.y &&
							 val.z > m_filterValc1.z );
				}
				case FILTER_EQUAL:
				{
					return ( val.x == m_filterValc1.x &&
							 val.y == m_filterValc1.y &&
							 val.z == m_filterValc1.z );
				}
				case FILTER_BETWEEN:
				{
					return ( ( val.x > m_filterValc1.x && val.x < m_filterValc2.x ) &&
							 ( val.y > m_filterValc1.y && val.y < m_filterValc2.y ) &&
							 ( val.z > m_filterValc1.z && val.z < m_filterValc2.z )  );
				}
				ERRV("Unknown value filter mode: ",m_filterMode);
			};
			break;
		}
		case RADIUS_FILTER:
		{
			switch( m_filterMode )
			{
				case FILTER_LESSTHAN:
				{
					if ( radius < m_filterValf1 ) return true;
					else return false;
				}
				case FILTER_MORETHAN:
				{
					if ( radius > m_filterValf1 ) return true;
					else return false;
				}
				case FILTER_EQUAL:
				{
					if ( radius == m_filterValf1 ) return true;
					else return false;
				}
				case FILTER_BETWEEN:
				{
					if ( radius > m_filterValf1 && radius < m_filterValf2 ) return true;
					else return false;
				}
				ERRV("Unknown filter mode: ",m_filterMode);
			};
			break;
		}
	};

	ERRV("Unknown filter variable: ",m_filterVariable);
	return false;
}


//
//  dnPtcViewerNode::draw
//
void dnPtcViewerNode::draw(   M3dView & view, const MDagPath & path,
							  M3dView::DisplayStyle style,
							  M3dView::DisplayStatus displaystatus )
{
	MStatus status;
	if ( !m_bDummyConnected || !m_bFilterChangeConnected ) connectDummyToShear();
	if ( !m_timeConnected ) connectToTime();

	int frame = ( int ) MAnimControl::currentTime().as( MTime::uiUnit() );

	MObject thisNode = thisMObject();
	MFnDependencyNode nodeFn(thisNode);

	// Start drawing
	//
	view.beginGL();
		glPushMatrix();
		glPushAttrib( GL_ALL_ATTRIB_BITS);


		// display the bounding box when selected.
		if ( displaystatus != M3dView::kDormant )
		{

			if ( m_doCropBox )
			{
				draw_cropBox( false );
				draw_bbox( true );
			}
			else
			{
				draw_cropBox( true );
				draw_bbox( false );
			}

		}

		bool vectorData = m_channelType == "vector" || m_channelType == "normal";

		if ( m_ptcID < 0 )
		{
			int slices = 8;
			if ( m_viewAs == 1 )
			{
				slices = m_circleSlices;
			}
			else
			{
				slices = m_diskSlices;
			}


			{
				m_diskID = glGenLists(1);
				glNewList( m_diskID, GL_COMPILE );
					if ( m_viewAs == 1 ) {
						if ( m_smooth ) {
							glEnable(GL_BLEND);
							glEnable(GL_LINE_SMOOTH);
							glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
						}
						gluQuadricDrawStyle(m_quadricObj,GLU_SILHOUETTE);
					} else if ( m_viewAs == 2 )
						gluQuadricDrawStyle(m_quadricObj,GLU_FILL);
					gluQuadricNormals(m_quadricObj,GLU_NONE);
					gluDisk( m_quadricObj, 0, 1.0, slices, 1 );
				glEndList();
				DEBUG(" + Create disk");
			}

			m_ptcID = glGenLists(1);

			glNewList( m_ptcID, GL_COMPILE_AND_EXECUTE );

				if ( m_smooth )
				{
					glEnable(GL_DEPTH_TEST);
					glDepthFunc(GL_LESS);
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
				}
				glEnable( GL_POINT_SMOOTH );


				glPointSize( m_ptSize );

				int len = m_pts.length();
				MVector col( 1.0, 1.0, 1.0 );

				if ( m_viewAs == 0 )
				{
					glBegin( GL_POINTS );


						if ( vectorData )
						{
							// NORMAL and VECTOR channels
							if ( m_invert ) glColor3f( 1.0, 1.0, 1.0 );
							else glColor3f( 0.5, 0.5, 0.5 );
							for ( int i=0; i<len; i++ )
							{
								glVertex3f( m_pts[i].x, m_pts[i].y, m_pts[i].z );
							}
						}
						else
						{
							// FLOAT and COLOR channels
							for ( int i=0; i<len; i++ )
							{
								col = m_ptsColor[i];
								if ( m_filterOn == FILTER_ON_LOAD ||
									filterPoint( col, m_ptsRadius[i], false ) )
								{
									if ( m_abs )        col = MVector( fabs(col.x), fabs(col.y), fabs(col.z) );
									if ( m_invert )     col = MVector( 1.0,1.0,1.0 ) - col;
									if ( m_exp != 1.0 ) col = m_exp * col;
									glColor3f( col.x, col.y, col.z );
									glVertex3f( m_pts[i].x, m_pts[i].y, m_pts[i].z );
								}
							}
						}
					glEnd();
				}
				else
				{
					if ( vectorData )
					{
						// NORMAL and VECTOR channels
						// do nothing
					}
					else
					{
						// FLOAT and COLOR channels
						float m[16];
						for ( int i=0; i<len; i++ )
						{
							col = m_ptsColor[i];
							if ( m_filterOn == FILTER_ON_LOAD ||
								filterPoint( col, m_ptsRadius[i], false ) )
							{
								if ( m_abs )        col = MVector( fabs(col.x), fabs(col.y), fabs(col.z) );
								if ( m_invert )     col = MVector( 1.0,1.0,1.0 ) - col;
								if ( m_exp != 1.0 ) col = m_exp * col;

								MVector n( m_ptsNormal[i].x, m_ptsNormal[i].y, m_ptsNormal[i].z );
								n.normalize();
								MVector ref(1,0,0);
								MVector up = ref ^ n;
								up.normalize();
								MVector right = up ^ n;
								right.normalize();

								m[0]  = right.x;
								m[1]  = right.y;
								m[2]  = right.z;
								m[3]  = 0.0;
								m[4]  = up.x;
								m[5]  = up.y;
								m[6]  = up.z;
								m[7]  = 0.0;
								m[8]  = n.x;
								m[9]  = n.y;
								m[10] = n.z;
								m[11] = 0.0;
								m[12] = m_pts[i].x;
								m[13] = m_pts[i].y;
								m[14] = m_pts[i].z;
								m[15] = 1.0;

								glPushMatrix();
									glMultMatrixf(m);
									glColor3f( col.x, col.y, col.z );
									glPushMatrix();
										glScalef(m_ptsRadius[i], m_ptsRadius[i], m_ptsRadius[i]);
										glCallList(m_diskID);
									glPopMatrix();
								glPopMatrix();
							}
						}
					}
				}

				if ( m_normals && !vectorData )
				{

					glBegin( GL_LINES );
						for ( int i=0; i<len; i++ )
						{
							if ( m_filterOn == FILTER_ON_LOAD ||
								filterPoint( m_ptsColor[i], m_ptsRadius[i], false ) )
							{
								if ( m_invertNormalsColor )
									glColor3f( -m_ptsNormal[i].x, -m_ptsNormal[i].y, -m_ptsNormal[i].z );
								else
									glColor3f( m_ptsNormal[i].x, m_ptsNormal[i].y, m_ptsNormal[i].z );

								glVertex3f( m_pts[i].x, m_pts[i].y, m_pts[i].z );
								glVertex3f( m_pts[i].x + ( m_ptsNormal[i].x * m_normalsSize ),
								m_pts[i].y + ( m_ptsNormal[i].y * m_normalsSize ),
								m_pts[i].z + ( m_ptsNormal[i].z * m_normalsSize ) );
							}
						}
					glEnd();
				}


				if ( vectorData )
				{
					glBegin( GL_LINES );
						for ( int i=0; i<len; i++ )
						{
							if ( m_invertNormalsColor )
								glColor3f( -m_ptsColor[i].x, -m_ptsColor[i].y, -m_ptsColor[i].z );
							else
								glColor3f( m_ptsColor[i].x, m_ptsColor[i].y, m_ptsColor[i].z );

							glVertex3f( m_pts[i].x, m_pts[i].y, m_pts[i].z );
							glVertex3f( m_pts[i].x + ( m_ptsColor[i].x * m_normalsSize ),
										m_pts[i].y + ( m_ptsColor[i].y * m_normalsSize ),
										m_pts[i].z + ( m_ptsColor[i].z * m_normalsSize ) );
						}
					glEnd();
				}
			glEndList();

		}
		else
		{
			DEBUG(" + use display list");
			glCallList( m_ptcID );
		}

		if ( m_showValue && !vectorData )
		{
			int len = m_pts.length();
			glColor3f(0.4,0.4,0.4);
			MString label;
			MPoint  lpos;
			for ( int i=0; i<len; i++ )
			{
				if ( m_filterOn == FILTER_ON_LOAD ||
					filterPoint( m_ptsColor[i], m_ptsRadius[i], false ) )
				{
					label = " ( ";
					label += m_ptsColor[i].x;
					label += ", ";
					label += m_ptsColor[i].y;
					label += ", ";
					label += m_ptsColor[i].z;
					label += " )";
					lpos.x  = m_pts[i].x;
					lpos.y  = m_pts[i].y;
					lpos.z  = m_pts[i].z;
					view.drawText( label, lpos, M3dView::kLeft );
				}
			}
		}

	  glPopAttrib();
	glPopMatrix();
  view.endGL();

}


//
//  dnPtcViewerNode::isBounded
//
bool dnPtcViewerNode::isBounded() const
{
	return true;
}


//
//  dnPtcViewerNode::boundingBox
//
MBoundingBox dnPtcViewerNode::boundingBox() const
{
	if ( m_doCropBox )
	{
		return m_cropBoxLocal;
	}
	return m_bbox;
}


//
//  dnPtcViewerNode::creator
//
void* dnPtcViewerNode::creator()
{
	return new dnPtcViewerNode();
}


//
//  dnPtcViewerNode::initialize
//
MStatus dnPtcViewerNode::initialize()
{
	MFnTypedAttribute	matAttr;
	MFnTypedAttribute	strAttr;
	MFnNumericAttribute	numAttr;
	MFnUnitAttribute	uAttr;
	MFnEnumAttribute	eAttr;
	MStatus				stat;

	aFilterChange = numAttr.create( "filterChange", "fic", MFnNumericData::kFloat, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setStorable(false) );
	CHECK_MSTATUS( numAttr.setKeyable(false) );
	CHECK_MSTATUS( numAttr.setHidden(true) );
	CHECK_MSTATUS( addAttribute( aFilterChange ) );

	aDummy = numAttr.create( "dummy", "dum", MFnNumericData::kFloat, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setStorable(false) );
	CHECK_MSTATUS( numAttr.setKeyable(false) );
	CHECK_MSTATUS( numAttr.setHidden(true) );
	CHECK_MSTATUS( addAttribute( aDummy ) );


	aPtcFile = strAttr.create( MString("ptcFile"), MString("ptc"), MFnData::kString, aPtcFile, &stat );
	CHECK_MSTATUS( strAttr.setStorable(true) );
	CHECK_MSTATUS( strAttr.setKeyable(false) );
	CHECK_MSTATUS( strAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aPtcFile ) );

	aChannels = numAttr.create( "channel", "ch", MFnNumericData::kInt, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aChannels ) );

	aViewAs = eAttr.create( "viewAs", "va", 0, &stat );
	CHECK_MSTATUS( eAttr.setStorable(true) );
	CHECK_MSTATUS( eAttr.setKeyable(true) );
	CHECK_MSTATUS( eAttr.setConnectable(true) );
	CHECK_MSTATUS( eAttr.addField("Points",			0) );
	CHECK_MSTATUS( eAttr.addField("Radius Circle",	1) );
	CHECK_MSTATUS( eAttr.addField("Radius Disk",	2) );
	CHECK_MSTATUS( addAttribute( aViewAs ) );


	aPercent = numAttr.create( "percentLoaded", "pl", MFnNumericData::kFloat, 100.0, &stat );
	CHECK_MSTATUS( numAttr.setMin( 0.0f ) );
	CHECK_MSTATUS( numAttr.setMax(  100.0f ) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aPercent ) );

	aExposure = numAttr.create( "exposure", "exp", MFnNumericData::kFloat, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setMin( -100.0f ) );
	CHECK_MSTATUS( numAttr.setMax(  100.0f ) );
	CHECK_MSTATUS( numAttr.setSoftMin( -10.0f ) );
	CHECK_MSTATUS( numAttr.setSoftMax(  10.0f ) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aExposure ) );

	aPointSize = numAttr.create( "pointSize", "ps", MFnNumericData::kFloat, 1.0, &stat );
	CHECK_MSTATUS( numAttr.setMin(    1.0f ) );
	CHECK_MSTATUS( numAttr.setMax(  100.0f ) );
	CHECK_MSTATUS( numAttr.setSoftMin(   0.001f ) );
	CHECK_MSTATUS( numAttr.setSoftMax(  10.0f ) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aPointSize ) );

	aSmooth = numAttr.create( "smooth", "sm", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aSmooth ) );

	aInvert = numAttr.create( "invert", "i", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aInvert ) );

	aAbsoluteValue = numAttr.create( "absoluteValue", "abs", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aAbsoluteValue ) );

	aShowNormals = numAttr.create( "showNormals", "sn", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aShowNormals ) );

	aNormalsSize = numAttr.create( "normalsSize", "ns", MFnNumericData::kFloat, 1.0, &stat );
	CHECK_MSTATUS( numAttr.setMin(  0.001f ) );
	CHECK_MSTATUS( numAttr.setMax(  10.0f ) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aNormalsSize ) );

	aInvertNormalsColor = numAttr.create( "negateNormalsColor", "nnc", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aInvertNormalsColor ) );

	aUseCropBox = numAttr.create( "useCropBox", "cb", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aUseCropBox ) );

	aCropBoxMin = numAttr.createPoint( "cropBoxMin", "cmn", &stat );
	CHECK_MSTATUS( numAttr.setDefault( 0.0, 0.0, 0.0 ) );
	CHECK_MSTATUS( numAttr.setMin( 0.0, 0.0, 0.0 ) );
	CHECK_MSTATUS( numAttr.setMax( 1.0, 1.0, 1.0 ) );
	CHECK_MSTATUS( numAttr.setStorable(    true    ) );
	CHECK_MSTATUS( numAttr.setKeyable(     true    ) );
	CHECK_MSTATUS( numAttr.setConnectable( true    ) );
	CHECK_MSTATUS( addAttribute( aCropBoxMin ) );

	aCropBoxMax = numAttr.createPoint( "cropBoxMax", "cmx", &stat );
	CHECK_MSTATUS( numAttr.setDefault( 1.0, 1.0, 1.0 ) );
	CHECK_MSTATUS( numAttr.setMin( 0.0, 0.0, 0.0 ) );
	CHECK_MSTATUS( numAttr.setMax( 1.0, 1.0, 1.0 ) );
	CHECK_MSTATUS( numAttr.setStorable(    true    ) );
	CHECK_MSTATUS( numAttr.setKeyable(     true    ) );
	CHECK_MSTATUS( numAttr.setConnectable( true    ) );
	CHECK_MSTATUS( addAttribute( aCropBoxMax ) );

	aTimeSync = numAttr.create( "timeSync", "ts", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aTimeSync ) );

	aTime = uAttr.create( "time", "tm", MFnUnitAttribute::kTime, 0.0 );
	uAttr.setReadable(true);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
	uAttr.setKeyable(true);
	uAttr.setConnectable(true);
	stat = addAttribute( aTime );

	aShowValue = numAttr.create( "showValue", "sv", MFnNumericData::kBoolean, false, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aShowValue ) );

	aFilterVariable = eAttr.create( "filterVariable", "fv", 0, &stat );
	CHECK_MSTATUS( eAttr.setStorable(true) );
	CHECK_MSTATUS( eAttr.setKeyable(true) );
	CHECK_MSTATUS( eAttr.setConnectable(true) );
	CHECK_MSTATUS( eAttr.addField("Channel",	0) );
	CHECK_MSTATUS( eAttr.addField("Radius",		1) );
	CHECK_MSTATUS( addAttribute( aFilterVariable ) );

	aFilterMode = eAttr.create( "filterMode", "fm", 0, &stat );
	CHECK_MSTATUS( eAttr.setStorable(true) );
	CHECK_MSTATUS( eAttr.setKeyable(true) );
	CHECK_MSTATUS( eAttr.setConnectable(true) );
	CHECK_MSTATUS( eAttr.addField("Off",			0) );
	CHECK_MSTATUS( eAttr.addField("v < v1",			1) );
	CHECK_MSTATUS( eAttr.addField("v > v1",			2) );
	CHECK_MSTATUS( eAttr.addField("v == v1",		3) );
	CHECK_MSTATUS( eAttr.addField("v1 < v < v2",	4) );
	CHECK_MSTATUS( addAttribute( aFilterMode ) );

	aFilterValf1 = numAttr.create( "filterValf1", "fvf1", MFnNumericData::kFloat, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aFilterValf1 ) );

	aFilterValf2 = numAttr.create( "filterValf2", "fvf2", MFnNumericData::kFloat, 1.0, &stat );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aFilterValf2 ) );

	aFilterValc1 = numAttr.createColor( "filterValc1", "fvc1", &stat );
	CHECK_MSTATUS( numAttr.setDefault(0.0, 0.0, 0.0) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aFilterValc1 ) );

	aFilterValc2 = numAttr.createColor( "filterValc2", "fvc2", &stat );
	CHECK_MSTATUS( numAttr.setDefault(1.0, 1.0, 1.0) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aFilterValc2 ) );

	aCircleSlices = numAttr.create( "circleSlices", "csl", MFnNumericData::kInt, 8, &stat );
	CHECK_MSTATUS( numAttr.setMin(3) );
	CHECK_MSTATUS( numAttr.setMax(32) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aCircleSlices ) );

	aDiskSlices = numAttr.create( "diskSlices", "dsl", MFnNumericData::kInt, 8, &stat );
	CHECK_MSTATUS( numAttr.setMin(3) );
	CHECK_MSTATUS( numAttr.setMax(32) );
	CHECK_MSTATUS( numAttr.setStorable(true) );
	CHECK_MSTATUS( numAttr.setKeyable(true) );
	CHECK_MSTATUS( numAttr.setConnectable(true) );
	CHECK_MSTATUS( addAttribute( aDiskSlices ) );

	aFilterOn = eAttr.create( "filterOn", "fo", 0, &stat );
	CHECK_MSTATUS( eAttr.setStorable(true) );
	CHECK_MSTATUS( eAttr.setKeyable(true) );
	CHECK_MSTATUS( eAttr.setConnectable(true) );
	CHECK_MSTATUS( eAttr.addField("Load",			0) );
	CHECK_MSTATUS( eAttr.addField("Draw",			1) );
	CHECK_MSTATUS( addAttribute( aFilterOn ) );

	// output attr
	aChannelNames = strAttr.create(
			MString("channelNames"),
			MString("chn"),
			MFnData::kStringArray,
			aChannelNames,
			&stat
			);
	CHECK_MSTATUS( strAttr.setHidden(      true  ) );
	CHECK_MSTATUS( strAttr.setStorable(    false ) );
	CHECK_MSTATUS( strAttr.setKeyable(     false ) );
	CHECK_MSTATUS( strAttr.setConnectable( true  ) );
	CHECK_MSTATUS( addAttribute( aChannelNames ) );

	aNumPoints = numAttr.create( "numPoints", "np", MFnNumericData::kInt, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setHidden(      true  ) );
	CHECK_MSTATUS( numAttr.setStorable(    false ) );
	CHECK_MSTATUS( numAttr.setKeyable(     false ) );
	CHECK_MSTATUS( numAttr.setConnectable( true  ) );
	CHECK_MSTATUS( addAttribute( aNumPoints ) );

	aNumPointsLoaded = numAttr.create( "numPointsLoaded", "npl", MFnNumericData::kInt, 0.0, &stat );
	CHECK_MSTATUS( numAttr.setHidden(      true  ) );
	CHECK_MSTATUS( numAttr.setStorable(    false ) );
	CHECK_MSTATUS( numAttr.setKeyable(     false ) );
	CHECK_MSTATUS( numAttr.setConnectable( true  ) );
	CHECK_MSTATUS( addAttribute( aNumPointsLoaded ) );


	// relationships
	//
	CHECK_MSTATUS( attributeAffects( aFilterVariable, aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterMode,     aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterValf1,    aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterValf2,    aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterValc1,    aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterValc2,    aFilterChange ) );
	CHECK_MSTATUS( attributeAffects( aFilterOn,       aFilterChange ) );

	CHECK_MSTATUS( attributeAffects( aPtcFile,            aDummy ) );
	CHECK_MSTATUS( attributeAffects( aChannels,           aDummy ) );
	CHECK_MSTATUS( attributeAffects( aViewAs,             aDummy ) );
	CHECK_MSTATUS( attributeAffects( aExposure,           aDummy ) );
	CHECK_MSTATUS( attributeAffects( aPointSize,          aDummy ) );
	CHECK_MSTATUS( attributeAffects( aSmooth,             aDummy ) );
	CHECK_MSTATUS( attributeAffects( aInvert,             aDummy ) );
	CHECK_MSTATUS( attributeAffects( aAbsoluteValue,      aDummy ) );
	CHECK_MSTATUS( attributeAffects( aShowNormals,        aDummy ) );
	CHECK_MSTATUS( attributeAffects( aNormalsSize,        aDummy ) );
	CHECK_MSTATUS( attributeAffects( aInvertNormalsColor, aDummy ) );
	CHECK_MSTATUS( attributeAffects( aPercent,            aDummy ) );
	CHECK_MSTATUS( attributeAffects( aUseCropBox,         aDummy ) );
	CHECK_MSTATUS( attributeAffects( aCropBoxMin,         aDummy ) );
	CHECK_MSTATUS( attributeAffects( aCropBoxMax,         aDummy ) );
	CHECK_MSTATUS( attributeAffects( aTimeSync,           aDummy ) );
	CHECK_MSTATUS( attributeAffects( aTime,               aDummy ) );
	CHECK_MSTATUS( attributeAffects( aShowValue,          aDummy ) );
	CHECK_MSTATUS( attributeAffects( aCircleSlices,       aDummy ) );
	CHECK_MSTATUS( attributeAffects( aDiskSlices,         aDummy ) );

	return MS::kSuccess;
}


//
//  dnPtcViewerNode::connectDummyToShear
//
MStatus dnPtcViewerNode::connectDummyToShear()
{
	MStatus status;

	MFnDagNode dagFn(thisMObject(), &status);
	CHECKERR(status,"dagFn");
	MObject parent = dagFn.parent(0, &status);
	if(parent.isNull())
		return MS::kFailure;

	dagFn.setObject(parent);
	MPlug shearXYPlug = dagFn.findPlug("shearXY",&status);
	CHECKERR(status,"dagFn.findPlug(shearXY)");
	MPlug dummyPlug = MPlug(thisMObject(), aDummy);
	if(!dummyPlug.isConnected())
	{
		MDGModifier mod;
		mod.connect( dummyPlug, shearXYPlug );
		mod.doIt();
		m_bDummyConnected = true;
	}
	MPlug shearXZPlug = dagFn.findPlug("shearXZ",&status);
	CHECKERR(status,"dagFn.findPlug(shearXZ)");
	MPlug filterChangePlug = MPlug(thisMObject(), aFilterChange);
	if(!filterChangePlug.isConnected())
	{
		MDGModifier mod;
		mod.connect( filterChangePlug, shearXZPlug );
		mod.doIt();
		m_bFilterChangeConnected = true;
	}
	return status;

}


//
//  dnPtcViewerNode::connectToTime
//
MStatus dnPtcViewerNode::connectToTime()
{
	MStatus status;

	// our time plug
	MPlug timePlug( thisMObject(), aTime );

	MSelectionList sList;
	MGlobal::getSelectionListByName("time1", sList);
	unsigned int nMatches = sList.length();
	if ( nMatches > 0 )
	{
		MObject timeDepObj;
		sList.getDependNode(0, timeDepObj);
		MFnDependencyNode timeDep( timeDepObj );
		MPlug outTimePlug = timeDep.findPlug("outTime",&status);
		CHECKERR(status,"timeDep.findPlug(outTime)");
		if ( !timePlug.isConnected() )
		{
			MDGModifier mod;
			mod.connect( outTimePlug, timePlug );
			mod.doIt();
			m_timeConnected = true;
		}
		else
		{
			m_timeConnected = true;
		}
	}

	return status;

}


//
//  dnPtcViewerNode::getPtcFrame
//
MString dnPtcViewerNode::getPtcFrame()
{
	if ( m_timeSync == false ) return m_ptcFile;

	MStatus stat;
	MStringArray split;
	stat = m_ptcFile.split('.',split);
	if ( stat == MS::kSuccess )
	{
		MString frame;
		MString finalPath;
		unsigned int l = split.length();
		for ( int i=l-1; i>=0; i-- )
		{
			if ( split[i].isInt() )
			{
				DEBUG("\nFRAME is "+split[i]);
				unsigned int padding = split[i].length();
				frame.set( m_time.value(), 0);
				while ( frame.length() < padding ) frame = "0" + frame;
				DEBUG("TIME is "+frame);
				split[i] = frame;
				break;
			}
		}
		finalPath = split[0];
		for ( unsigned i=1; i<l; i++ ) finalPath += "."+split[i];
		MFileObject fileObj;
		fileObj.setRawFullName(finalPath);
		if ( fileObj.exists() ) return finalPath;
	}
	return m_ptcFile;
}


//
//  dnPtcViewerNode::loadPtc
//
MStatus dnPtcViewerNode::loadPtc()
{
	MStatus stat;
	float point[3], normal[3], bbox[6];
	float radius;
	int datasize = 0;

	// local ptc caching caching
	//
	std::string ptcname = getPtcFrame().asChar();

	#ifdef _DEBUG
	cout <<endl<<endl<<"loadPtc : channel "<<m_chan<<" of "<<ptcname.c_str()<<endl;
	#endif

	// open the point cloud
	PtcPointCloud ptc = NULL;
	ptc = PtcSafeOpenPointCloudFile( const_cast< char* >( ptcname.c_str() ) );

	if ( ! ptc )
	{
		ERR("dnPtcViewerNode error: unable to open input file "+(MString)ptcname.c_str());
		return MS::kFailure;
	}

	int nChannels = 0;
	PtcGetPointCloudInfo( ptc, "npointvars", &nChannels );

	char **vartypes = NULL;
	PtcGetPointCloudInfo( ptc, "pointvartypes", &vartypes );

	char **varnames = NULL;
	PtcGetPointCloudInfo( ptc, "pointvarnames", &varnames );

	// avoid reading a channel that does not exist.
	//
	if ( m_chan >= nChannels ) m_chan = nChannels-1;

	// set the bounding box
	//
	PtcGetPointCloudInfo( ptc, "bbox", &bbox );
	m_bbox = MBoundingBox( MPoint( bbox[0], bbox[1], bbox[2] ), MPoint( bbox[3], bbox[4], bbox[5] ) );
	compute_cropBox_world();

	// set the channel names
	//
	#ifdef _DEBUG
	cout <<"Point cloud has "<<nChannels<<" channels"<<endl;
	#endif

	m_channelNames.clear();

	for ( int i=0; i<nChannels; i++ )
	{
		MString name = vartypes[i];
		name += " ";
		name += varnames[i];
		m_channelNames.append( name );
		DEBUG("  + "+name);
	}

	// find how many points are available
	//
	int nPoints = 0;
	PtcGetPointCloudInfo( ptc, "npoints",  &nPoints   );
	m_nPoints = nPoints;
	m_nPointsLoaded = 0;

	// allocate per-point data
	//
	PtcGetPointCloudInfo( ptc, "datasize", &datasize);
	DEBUG("datasize == "+datasize);
	boost::shared_array <float> data( new float[datasize] );

	#ifdef _DEBUG
	cout <<"Point cloud contains "<<nPoints<<" points"<<endl;
	#endif

	// which channel should we load ?
	//
	MString vType( vartypes[m_chan] );
	int isVector = ( vType == "float" )? 0:1;

	// set the current channel type
	//
	m_channelType = vType;

	// compute channel offset
	//
	int chanOffset = 0;
	for ( int ch=0; ch<m_chan; ch++ )
	{
		if ( MString(vartypes[ch]) == "float" )
			chanOffset += 1;
		else
			chanOffset += 3;
	}
	#ifdef _DEBUG
	cout <<"channel offset for "<<varnames[m_chan]<<" is "<<chanOffset<<" ( isVector == "<<isVector<<" : "<<vartypes[m_chan]<<" )"<<endl;
	#endif

	// clear the arrays
	//
	m_pts.clear();
	m_ptsColor.clear();
	m_ptsNormal.clear();
	m_ptsRadius.clear();

	#ifdef _DEBUG
	cout <<"m_cropBox = "<<m_cropBox.min()<<" "<<m_cropBox.max()<<endl;
	#endif

	// init progress window
	bool showProgress = false;
	float  progressStep = 100.0;
	if ( MGlobal::mayaState() ==  MGlobal::kInteractive )
		showProgress = MProgressWindow::reserve();

	if ( showProgress )
	{
		MProgressWindow:: setInterruptable(false);
		MString pmsg = "Loading ";
		pmsg += m_nPoints;
		pmsg += " points...";
		MProgressWindow:: setProgressStatus(pmsg);
		MProgressWindow:: setTitle("dnPtcViewer");
		MProgressWindow::setProgressMin(0);
		MProgressWindow::setProgressMax(m_nPoints);
		MProgressWindow::startProgress();
		progressStep = floor(m_nPoints / 100);
	}


	if ( m_doCropBox &&
		 !( m_cropBox.min() == MVector(0.0,0.0,0.0) && m_cropBox.max() == MVector(1.0,1.0,1.0) ) )
	{
		MVector pColor;

		#ifdef _DEBUG
		cout <<"reload bbox: condition is "<<( m_filterOn == FILTER_ON_DRAW)<<endl;
		#endif

		for ( int p=0; p<nPoints; p++ )
		{
			PtcReadDataPoint( ptc, point, normal, &radius, data.get() );
			if ( isInCropBox( point ) )
			{
				if ( isVector )
					pColor = MVector( data[chanOffset], data[chanOffset+1], data[chanOffset+2] );
				else
					pColor = MVector( data[chanOffset], data[chanOffset], data[chanOffset] );

				if ( filterPoint(pColor,radius,true) )
				{
					m_pts.append( MVector( point[0], point[1], point[2] ) );
					m_ptsRadius.append(radius);
					m_ptsNormal.append( MVector( normal[0], normal[1], normal[2] ) );
					m_ptsColor.append( pColor );
					m_nPointsLoaded++;
				}
			}
			if ( showProgress && ( fmod((float)p,progressStep) == 0.0 || p >= nPoints ) )
				MProgressWindow:: setProgress(p);
		}

	}
	else
	{

		// how many points are we going to load ?
		//
		float factor = m_percent * 0.01f;
		int numPts = (int) fmax( 1.0f, floorf((float) nPoints * factor) );

		#ifdef _DEBUG
		cout <<"Loading "<<numPts<<" of "<<nPoints<<" points ( "<<m_percent<<"% )"<<endl;
		#endif

		// allocate memory
		//

		#ifdef _DEBUG
		cout <<"Allocating "<<numPts<<" points..."<<endl;
		#endif

		stat = m_pts.setLength(numPts);
		if ( stat != MS::kSuccess )
		{
			MString err = "Memory allocation for ";
			err += numPts;
			err += " points failed";
			ERR(err);
			return MS::kFailure;
		}
		stat = m_ptsColor.setLength(numPts);
		if ( stat != MS::kSuccess )
		{
			MString err = "Memory allocation for ";
			err += numPts;
			err += " colors failed";
			ERR(err);
			return MS::kFailure;
		}
		stat = m_ptsNormal.setLength(numPts);
		if ( stat != MS::kSuccess )
		{
			MString err = "Memory allocation for ";
			err += numPts;
			err += " normals failed";
			ERR(err);
			return MS::kFailure;
		}
		stat = m_ptsRadius.setLength(numPts);
		if ( stat != MS::kSuccess )
		{
			MString err = "Memory allocation for ";
			err += numPts;
			err += " radius failed";
			ERR(err);
			return MS::kFailure;
		}

		float s = floor( float(nPoints) / float(numPts) );
		int sp = 0;
		MVector pColor;
		bool test = m_filterOn == FILTER_ON_DRAW;

		#ifdef _DEBUG
		cout <<"reload all:"<<endl;
		cout <<"   condition is "<<test<<" ( "<<m_filterOn<<" )"<<endl;
		cout <<"   filterVar is "<<m_filterVariable<<endl;
		cout <<"   filterMode is "<<m_filterMode<<endl;
		cout <<"   v1 = "<<m_filterValf1<<endl;
		cout <<"   v2 = "<<m_filterValf2<<endl;
		#endif

		MVector faraway = MVector(1000000.0,1000000.0,1000000.0);
		for ( int p=0; p<nPoints; p++ )
		{
			PtcReadDataPoint( ptc, point, normal, &radius, data.get() );

			m_pts.set( faraway, sp );

			if ( fmod((float)p,s) == 0.0 )
			{
				if ( sp < numPts )
				{
					if ( isVector )
						pColor = MVector( data[chanOffset], data[chanOffset+1], data[chanOffset+2] );
					else
						pColor = MVector( data[chanOffset], data[chanOffset], data[chanOffset] );

					if ( filterPoint(pColor,radius,true) )
					{
						m_pts.set( point, sp );
						m_ptsRadius.set( radius, sp );
						m_ptsNormal.set( normal, sp );
						m_ptsColor.set( pColor, sp );
						m_nPointsLoaded++;
					}
				}
				sp++;
			}

			if ( showProgress && ( fmod((float)p,progressStep) == 0.0 || p >= nPoints ) )  MProgressWindow:: setProgress(p);

		}
		#ifdef _DEBUG
		cout <<"   Loaded "<<m_nPointsLoaded<<" points !"<<endl;
		#endif
	}

	if ( showProgress )
	{
		MProgressWindow:: endProgress();
	}

	PtcClosePointCloudFile( ptc );


	return MS::kSuccess;
}








//
//  initializePlugin
//
MStatus initializePlugin( MObject obj )
{
	char *version = "$Revision: 19518 $";
	char ver[16];
	sscanf(version, "$Revision: %s",ver);

	MFnPlugin plugin( obj, "DNeg", ver, "any");

	CHECK_MSTATUS(
			plugin.registerNode(
					"dnPtcViewerNode",
					dnPtcViewerNode::id,
					&dnPtcViewerNode::creator,
					&dnPtcViewerNode::initialize,
					MPxNode::kLocatorNode
					)
			);

	MGlobal::executeCommandOnIdle( "rehash;eval \"source AEdnPtcViewerNodeTemplate.mel\";", false );

	return MS::kSuccess;
}


//
//  uninitializePlugin
//
MStatus uninitializePlugin( MObject obj)
{
	MFnPlugin plugin( obj );

	CHECK_MSTATUS( plugin.deregisterNode( dnPtcViewerNode::id ) );

	return MS::kSuccess;
}



