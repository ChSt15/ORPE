# /*************************************************************************//**
#  * @brief Description: Script to generate customer PA curve fits that map from
#  *   power levels to actual powers for the EFR32.
#  * @details
#  *   To use this file:
#  *   1. Load railtest configured to the desired PA onto your chip which
#  *      is mounted onto the board you are trying to characterize.
#  *   2. In railtest, run the "sweepTxPower" command. As you sweep through
#  *      the power levels, record the dBm output from a spectrum analyzer
#  *      into SubgigPowerMapping.csv or 2p4PowerMapping.csv, depending
#  *      on the PA you are trying to characterize.
#  *   3. Run this python script from the same directory as 2p4PowerMapping.csv
#  *      and SubgigPowerMapping.csv.
#  *   4. Make a copy of pa_curves_efr32.h, copy the results of this python
#  *      script into the appropriate macro in that file. If you only need one
#  *      PA, you only need to copy the data for that PA, not both.
#  *   5. Update HAL_PA_CURVE_HEADER to point your new file.
#  *****************************************************************************
#  * # License
#  * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
#  *****************************************************************************
#  *
#  * SPDX-License-Identifier: Zlib
#  *
#  * The licensor of this software is Silicon Laboratories Inc.
#  *
#  * This software is provided 'as-is', without any express or implied
#  * warranty. In no event will the authors be held liable for any damages
#  * arising from the use of this software.
#  *
#  * Permission is granted to anyone to use this software for any purpose,
#  * including commercial applications, and to alter it and redistribute it
#  * freely, subject to the following restrictions:
#  *
#  * 1. The origin of this software must not be misrepresented; you must not
#  *    claim that you wrote the original software. If you use this software
#  *    in a product, an acknowledgment in the product documentation would be
#  *    appreciated but is not required.
#  * 2. Altered source versions must be plainly marked as such, and must not be
#  *    misrepresented as being the original software.
#  * 3. This notice may not be removed or altered from any source distribution.
#  *
#  ****************************************************************************/

from __future__ import print_function
from pylab import *
import numpy
import math
import sys
import os

API_MAX_POWER = 20
API_MIN_POWER = -50

def FitAndPlotPower(actPower, yAxisValues, min=-100, max=100):
  filtPwr=[]
  filtYAxisValues=[]
  x = 0
  minYAxisValue = 10000
  maxYAxisValue = -1

  for pwr in actPower:
    # Select which key value pairs to use for this segment based on their power
    if (min - 1) <= pwr <= (max):
      filtPwr.append(actPower[x])
      filtYAxisValues.append(yAxisValues[x])
      if actPower[x] >= min and yAxisValues[x] < minYAxisValue:
        minYAxisValue = yAxisValues[x]
      if actPower[x] <= max and yAxisValues[x] > maxYAxisValue:
        maxYAxisValue = yAxisValues[x]

    x += 1

  # Do the actual curve fit
  if filtPwr:
    fig = plt.plot(filtPwr, filtYAxisValues)
    coefficients = numpy.polyfit(filtPwr, filtYAxisValues, 1)
    polynomial = numpy.poly1d(coefficients)
    ys = polynomial(filtPwr)
    fig = plt.plot(filtPwr, ys, label=polynomial)
  else:
    polynomial = [0, 0]

  return minYAxisValue, maxYAxisValue, polynomial

def GenCArrayFromPolys(polylist):
    # build the C array string
  curveSegments = []
  for i in range(0, len(polylist), 3):
    maxPowerLevel = polylist[i+1]
    pwrcoeff = polylist[i+2]
    curveSegments.append(CurveSegment(int(maxPowerLevel), int(pwrcoeff[1] * 100), int(pwrcoeff[0] * 1000)))
      
  return curveSegments

def StringFromCurveSegments(curveSegments):
  arrayStr = "{ "
  length = 0

  # For formatting purposes, go through and find the longest string so we know what to
  # pad to
  for i in range(0, len(curveSegments)):
    segment = curveSegments[i]
    pwrStr = "{{{}, {}, {}}},".format(segment.maxValue, segment.slope, segment.intercept)

    if len(pwrStr) > length:
      length = len(pwrStr)

  for i in range(0, len(curveSegments)):
    segment = curveSegments[i]
    pwrStr = "{{ {}, {}, {} }},".format(segment.maxValue, segment.slope, segment.intercept)

    # Apply the padding width to each line
    pwrStr = pwrStr.ljust(length + 5)

    # Align matrix rows
    if i > 0:
      pwrStr = "  " + pwrStr

    pwrStr += "\\\n"
    arrayStr += pwrStr

  # Format the last part of the string correctly
  while arrayStr[len(arrayStr) - 1] != '}':
    arrayStr = arrayStr[:-1]
  arrayStr += " }"

  return arrayStr

def AdjustMaxValues(curveSegments):
  for i in range(1, len(curveSegments)):
    if curveSegments[i].slope != curveSegments[i-1].slope and curveSegments[i-1].slope != 0 and curveSegments[i].slope != 0:
      # Adjust the max values so that they overlap where the curves actually intercept
      curveSegments[i].maxValue = min(curveSegments[i-1].maxValue,
                                      (curveSegments[i].slope * int((curveSegments[i-1].intercept - curveSegments[i].intercept)
                                        / (curveSegments[i].slope - curveSegments[i-1].slope)) + curveSegments[i].intercept + 500) / 1000)

  return curveSegments

def FitCharData(increment=4):

  if len(sys.argv) < 2:
    print("Usage: python pa_customer_curve_fits.py <CSV_FILE>")
    exit()
  
  fitResult = ProcessCharDataAndFindPoly(sys.argv[1], increment)

  cStr = ""
  cStr += '\nRAIL_TxPowerCurveSegment_t[] C Structure\n'
  cStr += StringFromCurveSegments(AdjustMaxValues(fitResult))
  cStr += '\n'

  print('\n' + cStr)

def ProcessCharDataAndFindPoly(filename, increment=4):
  data = ReadAndProcessCharData(filename)
  polys = CalcPowerPolys(data.pwrlvls, data.outpwrs, increment)
  return polys

def ReadAndProcessCharData(filename):
  chardata = numpy.loadtxt(filename, delimiter=',')
  pwrlvls = []
  outpwrs = []
  maxpower = None

  # Average powers in case users provide a list of dBm sample for each level
  for entry in chardata:
    pwrlvls.append(entry[0])
    avgpower = average(entry[1:len(entry)])
    outpwrs.append(avgpower)
    if maxpower == None or avgpower > maxpower:
      maxpower = avgpower

  if maxpower > API_MAX_POWER:
    for x in range(0, len(pwrlvls)):
      outpwrs[x] -= (maxpower - API_MAX_POWER)

  return PaData(pwrlvls, outpwrs, maxpower)

scriptPath = os.path.dirname(__file__)

class PaData():
  def __init__(self, pwrlvls=None, outpwrs=None, maxpower=None):
    self.pwrlvls = pwrlvls
    self.outpwrs = outpwrs
    self.maxpower = maxpower

class CurveSegment():
  def __init__(self, maxValue, slope, intercept):
    self.maxValue = maxValue
    self.slope = slope
    self.intercept = intercept

def CalcPowerPolys(yAxisValues, powers, increment):
  polylist = []
  pwr = API_MAX_POWER
  numberOfSegments = 8
  for x in range(0, numberOfSegments):
    lowerBound = pwr - increment
    if x == (numberOfSegments - 1):
      # Large negative number, want to avoid float('-inf') due to arithmetic in
      # FitAndPlotPower
      lowerBound = -99999
    minYAxisValue, maxYAxisValue, polycoeff = FitAndPlotPower(powers, yAxisValues, lowerBound, pwr)

    pwr -= increment
    polylist.append(int(minYAxisValue))
    polylist.append(int(maxYAxisValue))
    polylist.append(polycoeff)

    if pwr < API_MIN_POWER:
      break
  # Uncomment the following line to see a mapping from dBm to power levels visual inspection
  # show()
  return GenCArrayFromPolys(polylist)

if __name__ == '__main__':
  FitCharData()

