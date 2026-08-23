; Load function headers
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/wrf/WRFUserARW.ncl"

begin 

  ;Create Root Folder Pointer;;;;;;;;;;;;;;;;;;;;;;
  ;Make sure that ROOT and DATA(from runWRFcode are identical)
  ROOT = "/home/carlos/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/"

  FILELOCATION = "WRFV3/run/wrfles.nc"

  SAVE = ROOT + "Simulation/Raw_Data/"

  data = addfile(ROOT + FILELOCATION,"r") ; read in file
  ;data = addfile("/media/SW_Preload/Root/Georgia_Tech/WindMapper/WindModeling/WRFV3/run/wrfhillexample.nc","r")

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;GET TIMES IN FILE;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  times  = wrf_user_list_times(data)  ; get times in the file
  ntimes = dimsizes(times)         ; number of times in the file
  timestamps = new(ntimes,float) ; create new variable for timestamps

  do it = 0,ntimes-1,1 ;do loop to set default values
    timestamps(it) = 0
  end do

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


  ;;;;;;;;;;;;;;;;GET NUMBER OF GRID POINTS;;;;;;;

  ;dummy = wrf_user_getvar(data,"U",0)
  ;maxDim = min(dimsizes(dummy))

  ;print("Max Dimension")
  ;print(sprintf("%2.0f",maxDim))
  
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


  ;Get Terrain Data Also and z coordinates

   ;;;;;;;;;;;(PH+PHB)/9.81 = Z;;;;;;;;;;;;

   ;ph = wrf_user_getvar(data,"PH",time)
   ;phb = wrf_user_getvar(data,"PHB",time)
   ;zpressure = (ph + phb)/9.81
   ;dimzpressure = dimsizes(zpressure)
   ;format = sprintf("%3.0f",maxDim) + "f20.10"
   ;opt1 = True
   ;print("Saving Zpressure coord")
   ;opt1@fout = SAVE + "Zpressure.txt"
   ;write_matrix(zpressure(0:maxDim-1,1,0:maxDim-1),format,opt1)
   ;print("Dimensions of Zpressure")
   ;print(sprintf("%9.1f",dimzpressure))

   
;;;;;;FOR LOOP AROUND DATA TO SAVE DATA TO TXT FILES;;;;

dimensionsZ = new(3,integer)
dimensionsX = new(3,integer)
dimensionsY = new(3,integer)

do it = 0,ntimes-1,1
;do it = 0,0
   
  time = it

  timestamps(it) = it

  print("Saving File")
  print(sprintf("%2.0f",time))

  ;HGT is terrain height in meters dimensions are Xlong,Xlatitude
  ;U,V,W is the wind in m/s dimensions are X-coord,y-coord,z-coord 
  ; since this is a 2-d simulation the y-coord is obsolete

  u_in   = wrf_user_getvar(data,"U",time)    ; ua is u averaged to mass points
  v_in   = wrf_user_getvar(data,"V",time)    ; va is v averaged to mass points
  w_in   = wrf_user_getvar(data,"W",time)    ; vertical velocity
  
  ;u,v,w are 3D grids with coordinates in the form (z,y,x)

  ;Get Dimensions of U,V,W Parameters

  dimensionsU = dimsizes(u_in)
  dimensionsV = dimsizes(v_in)
  dimensionsW = dimsizes(w_in)

  ;I will assume that dimsizes = [zDim,yDim,xDim] where Dim is the dimension
  ;of the variable depending on the variable
 
  ;Print Dimensions

  ;print("Dimensions of U")
  ;print(sprintf("%9.1f",dimensionsU))n
  ;print("Dimensions of V")
  ;print(sprintf("%9.1f",dimensionsV))
  ;print("Dimensions of W")
  ;print(sprintf("%9.1f",dimensionsW))

  ;print("U")
  ;print(sprintf("%9.1f",u_in(0,0:4-1,0:9-1)))
  
  ;Save Dimensions to figure out maxDim in all directions

  xU = dimensionsU(2)
  xV = dimensionsV(2)
  xW = dimensionsW(2)
  yU = dimensionsU(1)
  yV = dimensionsV(1)
  yW = dimensionsW(1)
  zU = dimensionsU(0)
  zV = dimensionsV(0)
  zW = dimensionsW(0)

  ;print("zU")
  ;print(sprintf("%2.0f",zU))

  dimensionsZ(0) = zU
  dimensionsZ(1) = zV
  dimensionsZ(2) = zW
  dimensionsX(0) = xU
  dimensionsX(1) = xV
  dimensionsX(2) = xW
  dimensionsY(0) = yU
  dimensionsY(1) = yV
  dimensionsY(2) = yW

  maxZcoord = min(dimensionsZ)
  maxYcoord = min(dimensionsY)
  maxXcoord = min(dimensionsX)
 
  ;print("xcoord")
  ;print(sprintf("%2.0f",maxXcoord))
  ;print("ycoord")
  ;print(sprintf("%2.0f",maxYcoord))
  ;print("zcoord")
  ;print(sprintf("%2.0f",maxZcoord))

  ;Write Dimensions of Velocity field to text file
  ;asciiwrite("u_in.txt",u_in(0:39,0,0:201))

  ;U wind flags
  opt1 = True
  format1 = sprintf("%3.0f",maxXcoord) + "f20.15"

  ;V wind flags
  opt2 = True
  format2 = sprintf("%3.0f",maxXcoord) + "f20.15"

  ;W Wind flags
  opt3 = True
  format3 = sprintf("%3.0f",maxXcoord) + "f20.15"

  ;;Do loop

  do k = 0,maxZcoord-1,1

     ;Figure out Nomenclature for file
     ;zcoord_time.txt
     filestamp = k + "_"  + time + ".txt"

     opt1@fout = SAVE + "U" + filestamp
     write_matrix(u_in(k,0:maxYcoord-1,0:maxXcoord-1),format1,opt1)

     opt2@fout = SAVE + "V" + filestamp
     write_matrix(v_in(k,0:maxYcoord-1,0:maxXcoord-1),format2,opt2)

     opt3@fout = SAVE + "W" + filestamp
     write_matrix(w_in(k,0:maxYcoord-1,0:maxXcoord-1),format3,opt3)
     ;"9f10.4" means 9 columns 10 digits per variable and 4 digits after the decimal point

  end do

end do

;Output Timestamps
;First figure out how many timestamps to output

found = 0
output = -1
do it = 1,ntimes-1,1

  if (timestamps(it) .eq. 0) .and. (found .eq. 0) then

     found = 1
     output = it-1

  end if

end do

if (output .eq. -1) then

  output = ntimes-1

end if

asciiwrite(SAVE + "Times.txt",timestamps(0:output))

end

;;;;;;;;;;;;Z-COORD;;;;;;;;;;;;;;;;;;;;;

z_in = wrf_user_getvar(data,"z",0)
dimZ = dimsizes(z_in)
;print("Dimensions of Z")
;print(sprintf("%9.1f",dimZ))
format = sprintf("%3.0f",2) + "f20.10"
opt1 = True
print("Saving Z coord")
opt1@fout = SAVE + "Z.txt"
write_matrix(z_in(0:maxZcoord-1,0:1,1),format,opt1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;TERRAIN DATA;;;;;;;;;;;;;;;;;;;

HGT = wrf_user_getvar(data,"HGT",0)    ; terrain height in meters
dimHGT = dimsizes(HGT)
;print("Dimensions of HGT")
;print(sprintf("%9.1f",dimHGT))
opt = True
opt@fout = SAVE + "TerrainHeight.txt"
format = sprintf("%3.0f",dimHGT(1)) + "f10.4"
write_matrix(HGT(:,:),format,opt)    

;;;Output a file called Parameters that has X,Y and Zmax coord

maxcoord = new(3,float)
maxcoord(0) = maxXcoord
maxcoord(1) = maxYcoord
maxcoord(2) = maxZcoord
asciiwrite(SAVE + "Parameters.txt",maxcoord)



