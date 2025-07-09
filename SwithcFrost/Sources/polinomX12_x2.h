

  double polinom_x2(double x)
        {
            double y;
            double pol1 = -0.004593;
            double pol0 = 0.126;
            y = pol1 * Math.Pow(x, 2);
            y = y - pol0 * x -1.233;
            return y;

        }
				
	 double polinom_x12(double x)
        { double y;
            double pol12 = 2.537E-22; //E = 10^-6
            double pol11 = 3.519E-21;
            double pol10 = 3.212e-18 ;
            double pol9  = 4.449e-16 ;
            double pol8 =  1.498e-14;
            double pol7 =  6.543e-12;
            double pol6 =  1.583e-9;
            double pol5 =  2.306e-7;
            double pol4 =  7.512e-6;
            double pol3 =  5.83e-5;
            double pol2 =  0.004507;
            double pol1 =  0.02907;
            double pol0 = 0.186;// 0.185;// 0.268;
            y = pol12 * Math.Pow(x, 12);
            y = y - pol11 * Math.Pow(x, 11);
            y = y - pol10 * Math.Pow(x, 10);
            y = y - pol9 * Math.Pow(x, 9);
            y = y - pol8 * Math.Pow(x, 8);
            y = y + pol7 * Math.Pow(x,7);
            y = y + pol6 * Math.Pow(x,6 );
            y = y - pol5 * Math.Pow(x,5 );
            y = y + pol4 * Math.Pow(x,4);
            y = y + pol3 * Math.Pow(x,3 );
            y = y - pol2 * Math.Pow(x,2 );
            y = y + pol1 * x + pol0;



            if (x == 0)
                return y * 10e-3;
            else if (x > 0)
                return y * -1;
           else if (x <= -10)
             return   polinom_x2(x);
            else
                return y;
        }			