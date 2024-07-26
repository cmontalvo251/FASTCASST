function GetState(xin,t,tstar)

loc = find(tstar < t,1);
State = xin(:,loc)
