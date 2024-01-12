function A =  GAUSS(NRHS,NEQNS,A)
% C     
% C     MORAN - PAGE 78
% C     
% C     SOLUTION OF LINEAR ALGEBRAIC SYSTEM BY
% C     GAUSS ELIMINATION WITH PARTIAL PIVOTING
% C     
% C     [A] = COEFFICIENT MATRIX
% C     NEQNS = NUMBER OF EQUATIONS
% C     NRHS = NUMBER OF RIGHT-HAND SIDES
% C     
% C     RIGHT -HAND SIDES AND SOLUTIONS STORED IN
% C     COLUMNS NEQNS+1 THRU NEQNS+NRHS OF [A]
% C     
%      COMMON /COF/ A(5001,5002),NEQNS
NP = NEQNS + 1;
NTOT = NEQNS + NRHS;
% C     
% C     GAUSS REDUCTION
% C     
for I = 2:NEQNS
% C     
% C     -- SEARCH FOR LARGEST ENTRY IN (I-1)TH COLUMN
% C     ON OR BELOW MAIN DIAGONAL
% C     
  IM = I - 1;
  IMAX = IM;
  AMAX = abs(A(IM,IM));
  for J = I:NEQNS
    if (AMAX >= abs(A(J,IM))) 
    else
      IMAX = J;
      AMAX = abs(A(J,IM));
    end
  end
% C     
% C     -- SWITCH (I-1)TH AND IMAXTH EQUATIONS
% C     
  if (IMAX ~= IM)
  else
    for J = IM:NTOT
      TEMP = A(IM,J);
      A(IM,J) = A(IMAX,J);
      A(IMAX,J) = TEMP;
    end
% C     
% C     ELIMINATE (I -1)TH UNKNOWN FROM
% C     ITH THRU (NEQNS)TH EQUATIONS
% C   
  end
  for J = I:NEQNS
    R = A(J,IM)/A(IM,IM);
    for K = I:NTOT
      A(J,K) = A(J,K) - R*A(IM,K);
    end
  end
end

% C     
% C     BACK SUBSTITUTION
% C     
for K = NP:NTOT
  A(NEQNS,K) = A(NEQNS,K)/A(NEQNS,NEQNS);
  for L = 2:NEQNS
    I = NEQNS + 1 - L;
    IP = I + 1;
    for J = IP:NEQNS
      A(I,K) = A(I,K) - A(I,J)*A(J,K);
    end
    A(I,K) = A(I,K)/A(I,I);
  end
end
