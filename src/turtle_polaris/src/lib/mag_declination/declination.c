/* The Geomagnetism Library is used in this program. The program expects the files
WMM.COF and EGM9615.h to be in the same directory.

Author Samuel Garcin: https://github.com/francelico
January 24, 2021

Inspired by code written by:
Manoj.C.Nair@Noaa.Gov
November 15, 2009

 */
/****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <math.h>               /* for gcc */

#include "GeomagnetismHeader.h"
#include "EGM9615.h"


/* constants */
#define RECL 81

#define MAXINBUFF RECL+14

/** Max size of in buffer **/

#define MAXREAD MAXINBUFF-2
/** Max to read 2 less than total size (just to be safe) **/

#define PATH MAXREAD

/****************************************************************************/
/*                                                                          */
/*      Some variables used in this program                                 */
/*                                                                          */
/*    Name         Type                    Usage                            */
/* ------------------------------------------------------------------------ */
/*                                                                          */
/*   sdate  Scalar double           start date inputted                      */
/*                                                                          */
/*   alt        Scalar double           altitude above WGS84 Ellipsoid       */
/*                                                                          */
/*   latitude   Scalar double           Latitude.                            */
/*                                                                          */
/*   longitude  Scalar double           Longitude.                           */
/*                                                                          */
/*   inbuff     Char a of MAXINBUF     Input buffer.                        */
/*                                                                          */
/*   minyr      double                  Min year of all models               */
/*                                                                          */
/*   maxyr      double                  Max year of all models               */
/*                                                                          */
/*   yrmax      double array of MAXMOD  Max year of model.                   */
/*                                                                          */
/*   yrmin      double array of MAXMOD  Min year of model.                   */
/*                                                                          */

/****************************************************************************/

//TODO:
// 1. convert this into a function
// 2. try to call it in compiled file
// 3. implant in the Python file
float declination(double latitude, double longitude, double alt, double sdate, const char* libdir)
{
    /*  WMM Variable declaration  */

    MAGtype_MagneticModel *TimedMagneticModel, *MagneticModels[1];
    MAGtype_Ellipsoid Ellip;
    MAGtype_CoordSpherical CoordSpherical;
    MAGtype_CoordGeodetic CoordGeodetic;
    MAGtype_Date UserDate;
    MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
    MAGtype_Geoid Geoid;
    char ans[20];

    // get absolute filename for the coefficients
    char filename[] = "/WMM.COF";
    const size_t len1 = strlen(libdir);
    const size_t len2 = strlen(filename);
    char *abs_filename = malloc(len1 + len2 + 1); // +1 for the null-terminator
    memcpy(abs_filename, libdir, len1);
    memcpy(abs_filename + len1, filename, len2 + 1); // +1 to copy the null-terminator

    int NumTerms, epochs = 1, epoch = 0, i, nMax = 0, printErrors = 0;
    char VersionDate[12];

    char inbuff[MAXINBUFF];

    char args[7][MAXREAD];
    int iarg;

    double minyr;
    double maxyr;
    //double alt = -999999;
    //double sdate = -1;
    //double latitude = 200;
    //double longitude = 200;

    /* Initializations. */

    inbuff[MAXREAD + 1] = '\0'; /* Just to protect mem. */
    inbuff[MAXINBUFF - 1] = '\0'; /* Just to protect mem. */


    /* Memory allocation */

    strncpy(VersionDate, VERSIONDATE_LARGE + 39, 11);
    VersionDate[11] = '\0';
    
    if(!MAG_robustReadMagModels(abs_filename, &MagneticModels, epochs)) {
        do {
            printf("\n WMM.COF not found.  Press enter to exit... \n ");
        } while (NULL == fgets(ans, 20, stdin));
        return 1;
    }
    for(i = 0; i < epochs; i++) if(MagneticModels[i]->nMax > nMax) nMax = MagneticModels[i]->nMax;
    NumTerms = ((nMax + 1) * (nMax + 2) / 2);

    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */

    for(i = 0; i < epochs; i++) if(MagneticModels[i] == NULL || TimedMagneticModel == NULL)
        {
            MAG_Error(2);
        }

    MAG_SetDefaults(&Ellip, &Geoid); /* Set default values and constants */
    /* Check for Geographic Poles */

    /* Set EGM96 Geoid parameters */
    Geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    Geoid.Geoid_Initialized = 1;
    /* Set EGM96 Geoid parameters END */
    maxyr = MagneticModels[0]->CoefficientFileEndDate;
    minyr = MagneticModels[0]->epoch;

    // for(iarg = 0; iarg < argv; iarg++)
    //     if(argc[iarg] != NULL)
    //         strncpy(args[iarg], argc[iarg], MAXREAD);

    /** This will compute everything needed for 1 point in time. **/
    CoordGeodetic.lambda = longitude;
    CoordGeodetic.phi = latitude;
    CoordGeodetic.HeightAboveGeoid = alt;
    UserDate.DecimalYear = sdate;


    MAG_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &Geoid); /*This converts the height above mean sea level to height above the WGS-84 ellipsoid*/
    MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical); /*Convert from geodeitic to Spherical Equations: 17-18, WMM Technical report*/
    epoch = ((int) UserDate.DecimalYear - 1900) / 5;
    if(epoch >= epochs)
        epoch = epochs - 1;
    if(epoch < 0)
        epoch = 0;
    MAG_TimelyModifyMagneticModel(UserDate, MagneticModels[epoch], TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
    MAG_Geomag(Ellip, CoordSpherical, CoordGeodetic, TimedMagneticModel, &GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
    MAG_CalculateGridVariation(CoordGeodetic, &GeoMagneticElements);

    return GeoMagneticElements.Decl;

    }