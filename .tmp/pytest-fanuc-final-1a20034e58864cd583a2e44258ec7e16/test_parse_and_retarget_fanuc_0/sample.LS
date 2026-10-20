/PROG  SAMPLE
/MN
   0:  UFRAME_NUM=1 ;
   0:  UTOOL_NUM=1 ;
   1:J P[1] 25% FINE ;
   2:L P[2] 100mm/sec CNT50 ;
/POS
P[1]{
   GP1:
   UF : 1, UT : 1,
   J1= 10.000 deg, J2= -20.000 deg, J3= 30.000 deg,
   J4= 40.000 deg, J5= 50.000 deg, J6= 60.000 deg
};
P[2]{
   GP1:
   UF : 1, UT : 1, CONFIG : 'N U T, 0, 0, 0',
   X = 100.000 mm, Y = 200.000 mm, Z = 300.000 mm,
   W = 0.000 deg, P = 90.000 deg, R = 180.000 deg
};
/END
