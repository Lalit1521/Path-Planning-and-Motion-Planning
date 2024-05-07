#include "lalit_sridharan_roslab5/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include <complex>

using namespace std;


void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply2(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c/a - b*b/(3*a*a);
  float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3)/2);

  complex<float> inside = sqrt(q*q/4 + p*p*p/27);

  complex<float> root;

  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);
    //std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0) { return root; }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8*a*c-3*b*b)/(8*a*a);
  float q = (b*b*b-4*a*b*c+8*a*a*d)/(8*a*a*a);
  float r = (-3*b*b*b*b+256*a*a*a*e-64*a*a*b*d+16*a*b*b*c)/(256*a*a*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);

  complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;

  vector<complex<float>> roots { root1, root2, root3, root4 };

  float max_real_root = 0.f;

  for (complex<float> root: roots) {
    if(root.imag()==0){
    max_real_root = max(max_real_root, root.real());
  }
  //std::cout<<"Max real root:" << max_real_root<<std::endl;

  return max_real_root;
}}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans) {
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input : corresponds : a struct vector of Correspondene struct object defined in correspond.
  // input : curr_trans : A Transform object refernece
  // output : update the curr_trans object. Being a call by reference function, Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

// You can change the number of iterations here. More the number of iterations, slower will be the convergence but more accurate will be the results. You need to find the right balance.

int number_iter = 1;

for(int i = 0; i<number_iter; i++){

  //fill in the values of the matrics
  Eigen::MatrixXf M_i(2, 4);
  Eigen::Matrix2f C_i;
  Eigen::Vector2f pi_i;

  // Fill in the values for the matrices
  Eigen::Matrix4f M, W;
  Eigen::MatrixXf g(1, 4);
  M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  g << 0, 0, 0, 0;

  for (Correspondence c: corresponds)
  {
    pi_i << c.pj1->getVector();
    Eigen::Vector2f n_i;
    n_i = c.getNormalNorm();
    C_i = n_i * n_i.transpose();
    M_i << 1, 0, c.po->getX(), -c.po->getY(),
           0, 1, c.po->getY(), c.po->getX();
    M = M + M_i.transpose() * C_i * M_i;
    g = g - (2 * pi_i.transpose() * C_i * M_i);
  }

  // Define sub-matrices A, B, D from M
  M = 2 * M;
  Eigen::Matrix2f A, B, D;
  A << M.block(0, 0, 2, 2);
  B << M.block(0, 2, 2, 2);
  D << M.block(2, 2, 2, 2);

  //define S and S_A matrices from the matrices A B and D
  Eigen::Matrix2f S;
  Eigen::Matrix2f S_A;
  S << D-(B.transpose())*(A.inverse())*B;
  S_A << (S.determinant())*(S.inverse());


  //find the coefficients of the quadratic function of lambda
  float pow_2; float pow_1; float pow_0;
  Eigen::Matrix4f pow2_coeff, pow1_coeff, pow0_coeff;


  pow2_coeff << A.inverse()*B*B.transpose()*A.inverse().transpose(), -A.inverse()*B,
                (-A.inverse()*B).transpose(), Eigen::Matrix2f::Identity();
  pow1_coeff << A.inverse()*B*S_A*B.transpose()*A.inverse().transpose(), -A.inverse()*B*S_A,
                (-A.inverse()*B*S_A).transpose(), S_A;
  pow0_coeff << A.inverse()*B*S_A.transpose()*S_A*B.transpose()*A.inverse().transpose(), -A.inverse()*B*S_A.transpose()*S_A,
                (-A.inverse()*B*S_A.transpose()*S_A).transpose(), S_A.transpose()*S_A;
  
  //cout << -A.inverse()*B;
  //ROS_INFO("transpose");
  //cout<< (-A.inverse()*B).transpose().transpose();

  pow_2 = (4*g*pow2_coeff*g.transpose())(0);
  pow_1 = (4*g*pow1_coeff*g.transpose())(0);
  pow_0 = (g*pow0_coeff*g.transpose())(0);

  float c1, c2, c3, c4;
  c1 = S(0,0);
  c2 = S(0,1);
  c3 = S(1,0);
  c4 = S(1,1);

  float a, b, c, d, e;
  a = 16;
  b = 16*(c1+c4);
  c = (8*(c1*c4 - c2*c3)) + 4*(pow((c1+c4), 2)) - pow_2;
  d = 4*(c1+c4)*(c1*c4 - c2*c3) - pow_1;
  e = pow((c1*c4 - c2*c3), 2) - pow_0;


  // find the value of lambda by solving the equation formed. You can use the greatest real root function
  float lambda = greatest_real_root(a, b, c, d, e);

  //find the value of x which is the vector for translation and rotation
  Eigen::Vector4f x;
  x = -((M + 2*lambda*W).inverse()).transpose()*(g.transpose());

  // Convert from x to new transform

  float theta = atan2(x(3), x(2));
  curr_trans= Transform(x(0), x(1), theta);
  //Eigen::Vector2f error;
  //float magnitude = 0;
  //for (Correspondence c:corresponds)
  //{
  //  Eigen::Vector2f old_points;
  //  Eigen::Vector2f new_points;
  //  Eigen::Vector2f t;
  //  Eigen::Matrix2f R;
  //  t << x(0), x(1);
  //  R << cos(theta), -sin(theta), sin(theta), cos(theta);
  //  old_points << c.pj1->getX(), c.pj1->getY();
  //  new_points << c.po->getX(), c.po->getY();
  //  error = old_points - (R*new_points + t);
  //  magnitude = magnitude + sqrt(error[0]*error[0] + error[1]*error[1]);
  //}
  //return magnitude;
}
}

float updateTransform2(vector<Correspondence>& corresponds, Transform& curr_trans)
{
  Eigen::Vector2f y_o;
  Eigen::Vector2f x_o;
  y_o << 0, 0;
  x_o << 0, 0; 
  //Computing weighted average
  for (Correspondence c: corresponds)
  {
    Eigen::Vector2f old_points;
    Eigen::Vector2f new_points;
    old_points << c.pj1->getX(), c.pj1->getY();
    new_points << c.po->getX(), c.po->getY();
    //std::cout<<"old_points -" << old_points <<std::endl;
    //std::cout<<"new_points -" << new_points <<std::endl;
    y_o = y_o + old_points;
    x_o = x_o + new_points;
  }
  //std::cout<<"y_o -" << y_o <<std::endl;
  //std::cout<<"x_o -" << x_o <<std::endl;
  y_o = y_o/corresponds.size();
  x_o = x_o/corresponds.size();
  Eigen::Matrix2f H;
  H << 0, 0, 0, 0;
  //Computing Cross-covariance matrix
  for (Correspondence c:corresponds)
  {
    Eigen::Vector2f y_n;
    Eigen::Vector2f x_n;
    y_n << c.pj1->getX(), c.pj1->getY();
    x_n << c.po->getX(), c.po->getY();
    H = H + ((x_n - x_o) * (y_n - y_o).transpose()); 
  }
  //std::cout<<"y_o -" << y_o <<std::endl;
  //std::cout<<"x_o -" << x_o <<std::endl;
  //std::cout<<"H -" << H <<std::endl;
  //Computing svd for cross-covariance matrix
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2f U = svd.matrixU();
  Eigen::Matrix2f V = svd.matrixV();
  Eigen::Matrix2f R;
  R = V.transpose() * U.transpose();
  Eigen::Vector2f t;
  t = y_o - R*x_o;
  //std::cout<<"R -" << R <<std::endl;
  //std::cout<<"t -" << t <<std::endl;
  //Computing error
  Eigen::Vector2f error;
  float magnitude = 0;
  for (Correspondence c:corresponds)
  {
    Eigen::Vector2f old_points;
    Eigen::Vector2f new_points;
    old_points << c.pj1->getX(), c.pj1->getY();
    new_points << c.po->getX(), c.po->getY();
    error = old_points - (R*new_points + t);
    magnitude = magnitude + sqrt(error[0]*error[0] + error[1]*error[1]);
  }
  float theta = -atan2(R(1,0), R(0,0));
  curr_trans = Transform(t(0), t(1), theta);
  //ROS_INFO("pose: %f, %f, %f", t(0), t(1), theta);
  return magnitude;
} 

void updateTransform3(vector<Correspondence>& corresponds, Transform& curr_trans)
{
  int number_iter = 1;
  for(int i = 0; i<number_iter; i++)
  {

    //fill in the values of the matrics
    Eigen::MatrixXf M_i(2, 4);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;

    // Fill in the values for the matrices
    Eigen::Matrix4f M, W;
    Eigen::MatrixXf g(4, 1);
    M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    g << 0, 0, 0, 0;

    for (Correspondence c: corresponds)
    {
      pi_i << c.pj1->getVector();
      Eigen::Vector2f n_i;
      n_i = c.getNormalNorm();
      C_i = n_i * n_i.transpose();
      M_i << 1, 0, c.po->getX(), -c.po->getY(),
            0, 1, c.po->getY(), c.po->getX();
      M = M + M_i.transpose() * C_i * M_i;
      g = g - (2 * pi_i.transpose() * C_i * M_i).transpose();
    }
    // Define sub-matrices A, B, D from M
    M = 2 * M;
    Eigen::Matrix2f A, B, D;
    A << M.block(0, 0, 2, 2);
    B << M.block(0, 2, 2, 2);
    D << M.block(2, 2, 2, 2);

    //define S and S_A matrices from the matrices A B and D
    Eigen::Matrix2f S;
    Eigen::Matrix2f S_A;
    S << D-(B.transpose())*(A.inverse())*B;
    S_A << (S.determinant())*(S.inverse());
    
    Eigen::MatrixXf g1(2,1); 
    Eigen::MatrixXf g2(2,1);
    g1 << g(0), g(1);  // Extract rows 0 and 1
    g2 << g(2), g(3);  // Extract rows 2 and 3
    //cout<<g1;
    
    Eigen::MatrixXf p7(3, 1);
    //cout<<g2;
    //cout<<g1.transpose()*(A.inverse()*B*4*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*4)*g2 + g2.transpose()*(4)*g2;
    //cout<<g1.transpose()*(A.inverse()*B*4*S_A*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*4*S_A)*g2 + g2.transpose()*(4*S_A)*g2;
    //cout<<g1.transpose()*(A.inverse()*B*S_A*S_A*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*S_A*S_A)*g2 + g2.transpose()*(S_A*S_A)*g2;
    
    p7 << g1.transpose()*(A.inverse()*B*4*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*4)*g2 + g2.transpose()*(4)*g2,
          g1.transpose()*(A.inverse()*B*4*S_A*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*4*S_A)*g2 + g2.transpose()*(4*S_A)*g2,
          g1.transpose()*(A.inverse()*B*S_A*S_A*B.transpose()*A.inverse())*g1 + 2*g1.transpose()*(-A.inverse()*B*S_A*S_A)*g2 + g2.transpose()*(S_A*S_A)*g2;
    
    //cout << p7;
    
    Eigen::MatrixXf p_lambda(3, 1);
    

    p_lambda << 4, (2*S(0,0)+2*S(1,1)), ((S(0,0)*S(0,0))-(S(1,0)*S(0,1)));
    //cout << p_lambda;
    
    Eigen::MatrixXf p_conv(5,1);
    int n = p_lambda.size();
    int m = n;
    std::vector<float> c(n+m-1);
    for (int i = 0; i < n; ++i) 
    {
        for (int j = 0; j < m; ++j) 
        {
          // Update the convolution array
          c[i + j] += static_cast<int>(1*(-p_lambda(i) * p_lambda(j))) % 998244353;
        }
    }

    p_conv << c[0], c[1], c[2], c[3], c[4];
    //cout << p_conv;
    
    int maxSize = std::max(p_lambda.size(), p_conv.size());
    Eigen::MatrixXf result(5,1);
    //result = Eigen::VectorXf::Zero(maxSize);
    //cout << result;
    //cout << maxSize;
    result << p_conv(0), p_conv(1), p_conv(2)+p_lambda(0), p_conv(3)+p_lambda(1), p_conv(4)+p_lambda(2);
    //result.head(p_lambda.size()) += p_lambda;
    
    //result.head(p_conv.size()) += p_conv;
    
    // find the value of lambda by solving the equation formed. You can use the greatest real root function
    float lambda = greatest_real_root(result(0), result(1), result(2), result(3), result(4));
  
    //find the value of x which is the vector for translation and rotation
    Eigen::Vector4f x;
    x = -(M + 2*lambda*W).inverse()*g;
    
    // Convert from x to new transform

    float theta = atan2(x(3), x(2));
    curr_trans= Transform(x(0), x(1), theta);
    
    //curr_trans= Transform(0, 0, 0);
}
}

