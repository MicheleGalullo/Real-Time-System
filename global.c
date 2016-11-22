#include "global.h"

/* define screen dimension */

const int scrx = 1024;
const int scry = 768;
const int sezx = 900;     /* small screen dimension */
const int sezy = 600;     /* small screen dimension */
const int sezxm = 300;    /* length of motor screen */
const int sezym = 233;    /* heigth of motor screen */


/* define buttons geometrics property */

const int rectx1 = 925;              /* x position for first watch button */
const int recty1 = 85;               /* y position for first watch button */

const int pidx1 = 100;               /* x position for first pid button */
const int pidy1 = 100;               /* y position for first pid button */

const int motparx1 = 150;            /* x position for first button of motor's parameter */
const int motpary1 = 150;            /* y position for first button of motor's parameter */

const int l = 78;                    /* rectangle length */
const int h = 40;                    /* rectangle heigth */
const int dh = 10;                   /* text distance */
const int d = 35;                    /* spacing */
const int dy = 280;                  /* sezy-(ctrmpy+r+2*d) = 157 */
const int dx = 248;                  /* sezxm-3*d/2 */

/* define motors geometrics property */

const int r = 100;                    /* radius of motors */
const int ctrmpx = 150;               /* x center of motor for position control */
const int ctrmpy = 133;               /* y center of motor for position control */

const int ctrmvx = 450;               /* x center of motor for velocity control */
const int ctrmvy = 133;               /* y center of motor for velocity control */

const int ctrmtx = 750;               /* x center of motor for torque control */
const int ctrmty = 133;               /* y center of motor for torque control */

/* ogni riquadro del motore ha dimensioni di 300*233 */






/***********************************************************************************************************************************************/
/*************************************************** Initialize Global Variable ****************************************************************/
/***********************************************************************************************************************************************/

int n_p = 0, n_v = 0, n_t = 0;                                    /* the current motor in array */
int np = 0, nv = 0, nt = 0;                                       /* the total number of motor */
int page = WORK_PAGE;                                                 /* varible for page control */


/**********************************************************************************************************************************************/
/**********************************************************************************************************************************************/
/**********************************************************************************************************************************************/


