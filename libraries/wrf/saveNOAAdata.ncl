;in order to run this code you need to have ncl-ncarg installed
;sudo apt-get install ncl-ncarg
;you probably want to run this through this tutorial as well
;http://cmrelabtutorials.blogspot.com/2015/04/weather-research-and-forecasting-model.html?view=sidebar
;
;then just invoke ncl saveNOAAdata.ncl

; Load function headers
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/wrf/WRFUserARW.ncl"

UFILELOCATION = "Wind_Data/uwnd.sig995.2011.nc"
VFILELOCATION = "Wind_Data/vwnd.sig995.2011.nc"

SAVE = "Compiled_Data/"

dataU = addfile(UFILELOCATION,"r") ;read in file
dataV = addfile(VFILELOCATION,"r") ;read in file

;GET TIMES IN FILE;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ntimes = 365   ; number of times in the file
timestamps = new(ntimes,float) ; create new variable for timestamps

do it = 0,ntimes-1,1 ;do loop to set default values
   timestamps(it) = 0
   times  = wrf_user_getvar(dataU,"time",it)  ; get times in the file
end do

dummy = wrf_user_getvar(dataU,"uwnd",0)
dims = dimsizes(dummy)
lat = dims(0)
lon = dims(1)
maxXcoord = max(dims)

do it = 0,ntimes-1,1
  time = it
  timestamps(it) = it
  print(sprintf("%2.0f",time))
  uwnd = wrf_user_getvar(dataU,"uwnd",time)
  vwnd = wrf_user_getvar(dataV,"vwnd",time)
  ;dimensionsU = dimsizes(uwnd)
  ;print("Dimensions of U")
  ;print(sprintf("%9.1f",dimensionsU))

  ;U wind flags
  opt1 = True
  opt1@fout = SAVE + "V" + time + ".txt"
  format1 = sprintf("%3.0f",lon) + "I10"
  write_matrix(vwnd(0:lat-1,0:lon-1),format1,opt1)

  ;V wind flags
  opt2 = True
  opt2@fout = SAVE + "U" + time + ".txt"
  format2 = sprintf("%3.0f",lon) + "I10"
  write_matrix(uwnd(0:lat-1,0:lon-1),format2,opt2)

end do



