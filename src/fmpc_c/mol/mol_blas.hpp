#include "mol.hpp"

/* SIDE:    0 = Left side   -> A*x
 *          1 = Right side  -> x*A
 * At:      0, A is not Not transposed
 *          1, A is transposed
 * UPLO:    0, upper
 *          1, lower
 */
#define MOL_UP_TRI 0
#define MOL_LO_TRI 1

#define MOL_NO_TRANS 0
#define MOL_TRANS 1

#define MOL_MAT_LEFT 0
#define MOL_MAT_RIGHT 1

#include "faxpy.hpp"
#include "fdot.hpp"
#include "fgemm.hpp"
#include "fgemv.hpp"
#include "fnrm2.hpp"
#include "fposv.hpp"
#include "fpotf2.hpp"
#include "fpotrs.hpp"
#include "fscal.hpp"
#include "fsyrk.hpp"
#include "ftrsm.hpp"
#include "ftrtrs.hpp"





