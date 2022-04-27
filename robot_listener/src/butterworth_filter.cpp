// This code was inspired by: https://github.com/nxsEdson/Butterworth-Filter
#include <robot_listener/butterworth_filter.hpp>

using namespace std;

#define PI 3.14159

vector<double> ButterworthFilter::ComputeDenCoeffs(int FilterOrder, double Lcutoff, double Ucutoff)
{
	int k;            // loop variables
	double theta;     // PI * (Ucutoff - Lcutoff) / 2.0
	double cp;        // cosine of phi
	double st;        // sine of theta
	double ct;        // cosine of theta
	double s2t;       // sine of 2*theta
	double c2t;       // cosine 0f 2*theta
	vector<double> RCoeffs(2 * FilterOrder);     // z^-2 coefficients 
	vector<double> TCoeffs(2 * FilterOrder);     // z^-1 coefficients
	vector<double> DenomCoeffs;     // dk coefficients
	double PoleAngle;      // pole angle
	double SinPoleAngle;     // sine of pole angle
	double CosPoleAngle;     // cosine of pole angle
	double a;         // workspace variables

	cp = cos(PI * (Ucutoff + Lcutoff) / 2.0);
	theta = PI * (Ucutoff - Lcutoff) / 2.0;
	st = sin(theta);
	ct = cos(theta);
	s2t = 2.0*st*ct;        // sine of 2*theta
	c2t = 2.0*ct*ct - 1.0;  // cosine of 2*theta

	for (k = 0; k < FilterOrder; ++k)
	{
		PoleAngle = PI * (double)(2 * k + 1) / (double)(2 * FilterOrder);
		SinPoleAngle = sin(PoleAngle);
		CosPoleAngle = cos(PoleAngle);
		a = 1.0 + s2t*SinPoleAngle;
		RCoeffs[2 * k] = c2t / a;
		RCoeffs[2 * k + 1] = s2t*CosPoleAngle / a;
		TCoeffs[2 * k] = -2.0*cp*(ct + st*SinPoleAngle) / a;
		TCoeffs[2 * k + 1] = -2.0*cp*st*CosPoleAngle / a;
	}

	DenomCoeffs = TrinomialMultiply(FilterOrder, TCoeffs, RCoeffs);

	DenomCoeffs[1] = DenomCoeffs[0];
	DenomCoeffs[0] = 1.0;
	for (k = 3; k <= 2 * FilterOrder; ++k)
		DenomCoeffs[k] = DenomCoeffs[2 * k - 2];

	for (int i = DenomCoeffs.size() - 1; i > FilterOrder * 2 + 1; i--)
		DenomCoeffs.pop_back();

	return DenomCoeffs;
}

vector<double> ButterworthFilter::TrinomialMultiply(int FilterOrder, vector<double> b, vector<double> c)
{
	int i, j;
	vector<double> RetVal(4 * FilterOrder);

	RetVal[2] = c[0];
	RetVal[3] = c[1];
	RetVal[0] = b[0];
	RetVal[1] = b[1];

	for (i = 1; i < FilterOrder; ++i)
	{
		RetVal[2 * (2 * i + 1)] += c[2 * i] * RetVal[2 * (2 * i - 1)] - c[2 * i + 1] * RetVal[2 * (2 * i - 1) + 1];
		RetVal[2 * (2 * i + 1) + 1] += c[2 * i] * RetVal[2 * (2 * i - 1) + 1] + c[2 * i + 1] * RetVal[2 * (2 * i - 1)];

		for (j = 2 * i; j > 1; --j)
		{
			RetVal[2 * j] += b[2 * i] * RetVal[2 * (j - 1)] - b[2 * i + 1] * RetVal[2 * (j - 1) + 1] +
				c[2 * i] * RetVal[2 * (j - 2)] - c[2 * i + 1] * RetVal[2 * (j - 2) + 1];
			RetVal[2 * j + 1] += b[2 * i] * RetVal[2 * (j - 1) + 1] + b[2 * i + 1] * RetVal[2 * (j - 1)] +
				c[2 * i] * RetVal[2 * (j - 2) + 1] + c[2 * i + 1] * RetVal[2 * (j - 2)];
		}

		RetVal[2] += b[2 * i] * RetVal[0] - b[2 * i + 1] * RetVal[1] + c[2 * i];
		RetVal[3] += b[2 * i] * RetVal[1] + b[2 * i + 1] * RetVal[0] + c[2 * i + 1];
		RetVal[0] += b[2 * i];
		RetVal[1] += b[2 * i + 1];
	}

	return RetVal;
}

vector<double> ButterworthFilter::ComputeNumCoeffs(int FilterOrder, double Lcutoff, double Ucutoff, vector<double> DenC)
{
	vector<double> TCoeffs;
	vector<double> NumCoeffs(2 * FilterOrder + 1);
	vector<complex<double>> NormalizedKernel(2 * FilterOrder + 1);

	vector<double> Numbers;
	for (double n = 0; n < FilterOrder * 2 + 1; n++)
		Numbers.push_back(n);
	int i;

	TCoeffs = ComputeHP(FilterOrder);

	for (i = 0; i < FilterOrder; ++i)
	{
		NumCoeffs[2 * i] = TCoeffs[i];
		NumCoeffs[2 * i + 1] = 0.0;
	}
	NumCoeffs[2 * FilterOrder] = TCoeffs[FilterOrder];

	double cp[2];
	double Bw, Wn;
	cp[0] = 2 * 2.0*tan(PI * Lcutoff / 2.0);
	cp[1] = 2 * 2.0*tan(PI * Ucutoff / 2.0);

	Bw = cp[1] - cp[0];
	//center frequency
	Wn = sqrt(cp[0] * cp[1]);
	Wn = 2 * atan2(Wn, 4);
	double kern;
	const std::complex<double> result = std::complex<double>(-1, 0);

	for (int k = 0; k< FilterOrder * 2 + 1; k++)
	{
		NormalizedKernel[k] = std::exp(-sqrt(result)*Wn*Numbers[k]);
	}
	double b = 0;
	double den = 0;
	for (int d = 0; d < FilterOrder * 2 + 1; d++)
	{
		b += real(NormalizedKernel[d] * NumCoeffs[d]);
		den += real(NormalizedKernel[d] * DenC[d]);
	}
	for (int c = 0; c < FilterOrder * 2 + 1; c++)
	{
		NumCoeffs[c] = (NumCoeffs[c] * den) / b;
	}

	for (int i = NumCoeffs.size() - 1; i > FilterOrder * 2 + 1; i--)
		NumCoeffs.pop_back();

	return NumCoeffs;
}

vector<double> ButterworthFilter::ComputeLP(int FilterOrder)
{
	vector<double> NumCoeffs(FilterOrder + 1);
	int m;
	int i;

	NumCoeffs[0] = 1;
	NumCoeffs[1] = FilterOrder;
	m = FilterOrder / 2;
	for (i = 2; i <= m; ++i)
	{
		NumCoeffs[i] = (double)(FilterOrder - i + 1)*NumCoeffs[i - 1] / i;
		NumCoeffs[FilterOrder - i] = NumCoeffs[i];
	}
	NumCoeffs[FilterOrder - 1] = FilterOrder;
	NumCoeffs[FilterOrder] = 1;

	return NumCoeffs;
}

vector<double> ButterworthFilter::ComputeHP(int FilterOrder)
{
	vector<double> NumCoeffs;
	int i;

	NumCoeffs = ComputeLP(FilterOrder);

	for (i = 0; i <= FilterOrder; ++i)
		if (i % 2) NumCoeffs[i] = -NumCoeffs[i];

	return NumCoeffs;
}

vector<double> ButterworthFilter::filter(deque<double>x, vector<double> coeff_b, vector<double> coeff_a)
{
	int len_x = x.size();
	int len_b = coeff_b.size();
	int len_a = coeff_a.size();

	vector<double> zi(len_b);

	vector<double> filter_x(len_x);

	if (len_a == 1)
	{
		for (int m = 0; m<len_x; m++)
		{
			filter_x[m] = coeff_b[0] * x[m] + zi[0];
			for (int i = 1; i<len_b; i++)
			{
				zi[i - 1] = coeff_b[i] * x[m] + zi[i];//-coeff_a[i]*filter_x[m];
			}
		}
	}
	else
	{
		for (int m = 0; m<len_x; m++)
		{
			filter_x[m] = coeff_b[0] * x[m] + zi[0];
			for (int i = 1; i<len_b; i++)
			{
				zi[i - 1] = coeff_b[i] * x[m] + zi[i] - coeff_a[i] * filter_x[m];
			}
		}
	}

	return filter_x;
}


vector<double> ButterworthFilter::filter(vector<double>x, vector<double> coeff_b, vector<double> coeff_a)
{
	return {};
}

int main()
{
	ifstream ifile;
	ifile.open("/home/mads/git/project_in_advanced_robotics/wrench.csv");
	vector<vector<double>> input, output;


    std::string line, value;
    int i = 0;
    vector<double> vec_line;
    while(std::getline(ifile, line, '\n'))
    {
        for (int j = 0; j < 7; j++)
        {
            vec_line.push_back(stod(line.substr(0,line.find(","))));
            line.erase(0,(int)line.find(",")+1);
        }
        vec_line.push_back(stod(line));
        input.push_back(vec_line);
        vec_line.clear();
    }

    // for (int j = 0; j < input.size(); j++)
    // {
    //     cout << input[j][0] << ", " << input[j][1] << ", " << input[j][2] << ", " << input[j][3] << ", " << input[j][4] << ", " << input[j][5] << ", " << input[j][6] << ", " << input[j][7] << endl;
    // }

    

	double fps = 500, fc = 2;
	

	const int N = input.size();
	std::cout << N << std::endl;
	//	
	//for (int i = 0; i < 1000; i++)
	//{
	//	float x;
	//	ifile >> x;
	//	input.push_back(x);
	//}

	//Frequency bands is a vector of values - Lower Frequency Band and Higher Frequency Band

	//First value is lower cutoff and second value is higher cutoff
	double FrequencyBands[2] = { 0.000015,fc/(fps/2) };//these values are as a ratio of f/fs, where fs is sampling rate, and f is cutoff frequency
	//and therefore should lie in the range [0 1]
	//Filter Order

	int FiltOrd = 3;

	//Pixel Time Series
	/*int PixelTimeSeries[N];
	int outputSeries[N];
	*/
	//Create the variables for the numerator and denominator coefficients
	vector<double> a;
	vector<double> b;
	//Pass Numerator Coefficients and Denominator Coefficients arrays into function, will return the same

	deque<double> x(N);
	vector<double> y(N);

	for (int i = 0; i < N; i++)
	{
		x[i] = input[i][4];
	}
	
	ButterworthFilter filter;

	//is A in matlab function and the numbers are correct
	a = filter.ComputeDenCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1]);
	for (int k = 0; k<a.size(); k++)
	{
		printf("DenC is: %lf\n", a[k]);
	}

	b = filter.ComputeNumCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1], a);
	for (int k = 0; k<b.size(); k++)
	{
		printf("NumC is: %lf\n", b[k]);
	}


	std::deque<double> in;
	for (unsigned int i = 0; i < N; ++i)
	{
		if (i >= 1000)
		{
			in.pop_front();
			//std::cout << i << std::endl;
		}
		in.push_back(x[i]);
		//std::cout << in.back() << std::endl;
		std::vector<double> output = filter.filter(in,b,a);
		//std::cout << output.back() << std::endl;
		y[i] = output.back();
	}
	std::cout << "done calculating" << std::endl;
	//y = filter.filter(x, b, a);
	ofstream ofile;
	ofile.open("/home/mads/git/project_in_advanced_robotics/Filtered3.txt");
	for (int i = 0; i < N; i++)
	{
		ofile << y[i] << " " << x[i] << endl;
	}
	ofile.close();

	return 0;
}






