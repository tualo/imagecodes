

//#include <opencv2/objdetect.hpp>
//#include <opencv2/imgcodecs.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <zbar.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <boost/thread/scoped_thread.hpp>

#include "args.hxx"
#include "FindCodes.h"


#define HAVE_UINT64 1
#define HAVE_INT64 1
#define NO_LONGLONG 1

#undef min
#undef max
#undef test

#define HAVE_BOOL 1

#include <mysql.h>
#include <my_global.h>


bool showDebug= false;
bool showDebugWindow= false;
int windowWait=1000;

MYSQL *con;

void showImage(cv::Mat& src){
  if (showDebugWindow){
    cv::Mat rotated=src.clone();

    int x=src.cols /5;
    int y=src.rows /5;
    if ((x>0) && (y>0)){
      cv::Mat res = cv::Mat(x, y, CV_8UC1);
      cv::resize(rotated, res, cv::Size(x, y), 0, 0, 3);
      cv::namedWindow("DEBUG", CV_WINDOW_AUTOSIZE );
      cv::imshow("DEBUG", res );
      cv::waitKey(windowWait);
    }else{
      std::cerr << "ImageRecognizeEx::showImage can't show image" << std::endl;
    }
  }
}

int main(int argc, char* argv[])
{


    int exitCode = 0;
    args::ArgumentParser parser("Ocrs reconize barcodes live from camera.", "");
    args::HelpFlag help(parser, "help", "Display this help menu", { "help"});
    args::ValueFlag<std::string> filename_param(parser, "filename", "The machine ID", {'f',"filename"});
    args::Flag debug(parser, "debug", "Show debug messages", {'d', "debug"});
    args::Flag window(parser, "window", "Show debug windows", {'w', "window"});

    args::ValueFlag<std::string> resultpath(parser, "resultpath", "resultpath", {'r',"resultpath"});


    args::ValueFlag<std::string> db_host(parser, "host", "The database server host", {'h',"host"});
    args::ValueFlag<std::string> db_name(parser, "name", "The database name", {'n',"name"});
    args::ValueFlag<std::string> db_user(parser, "user", "The database server username", {'u',"user"});
    args::ValueFlag<std::string> db_pass(parser, "password", "The database server password", {'x',"password"});
    args::ValueFlag<std::string> db_encoding(parser, "encoding", "The database server encoding", {'e',"encoding"});


    try{
        parser.ParseCLI(argc, argv);
    }catch (args::Help){
        std::cout << parser;
        return 0;
    }catch (args::ParseError e){
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    if (window==1){
        showDebugWindow=true;
    }
    if (debug==1){
        showDebug=true;
    }


    FindCodes *fc = new FindCodes();
    fc->detect(args::get(filename_param));
    if (showDebug){
        fc->debugCodes();
    }
    std::string basename = (boost::filesystem::path(args::get(filename_param)).filename()).string();
    
    if (db_host){
      if (db_user){
        if (db_name){
         if (db_pass){
            std::string customer="";
            std::string state="99";

            boost::format set_camera_images_fmt("insert into camera_images (id,inserttime,kunde,state) values ('%s',now(),'%s','%s');  ");
            boost::format set_camera_imagescodes_fmt("insert into camera_imagescodes (id,code) values ('%s','%s') on duplicate key update id=values(id)");

            

            con = mysql_init(NULL);
            //str_db_encoding.c_str()
            mysql_options(con, MYSQL_SET_CHARSET_NAME, "utf8");
            mysql_options(con, MYSQL_INIT_COMMAND, "SET NAMES utf8");

            if (con == NULL){
                fprintf(stderr, "%s\n", mysql_error(con));
                exit(1);
            }

            if (mysql_real_connect(con, args::get(db_host).c_str(),args::get(db_user).c_str(), args::get(db_pass).c_str(),   args::get(db_name).c_str(), 0, NULL, 0) == NULL){
                fprintf(stderr, "%s\n", mysql_error(con));
                mysql_close(con);
                exit(1);
            }

            std::string sql = boost::str(set_camera_images_fmt % basename % customer % state );
            std::cout << std::endl << "====================="  << std::endl  << sql << std::endl << "=====================" <<  std::endl;
            if (mysql_query(con, sql.c_str())){
                fprintf(stderr, "%s\n", mysql_error(con));
                exit(1);
            }


            std::list<Barcode*> barcodes = fc->codes();


            std::list<Barcode*>::const_iterator it;
            for (it = barcodes.begin(); it != barcodes.end(); ++it){

                std::string sql = boost::str(set_camera_imagescodes_fmt % basename % ((Barcode*)*it)->code() );
                std::cout << std::endl << "====================="  << std::endl  << sql << std::endl << "=====================" <<  std::endl;
                if (mysql_query(con, sql.c_str())){
                    fprintf(stderr, "%s\n", mysql_error(con));
                    exit(1);
                }

            }
         }
        } 
      }
    }


    if (resultpath){
        try {
            boost::format result_path("%s%s");
            std::string savefile = boost::str(result_path % args::get(resultpath) % basename  );
            std::rename((args::get(filename_param)).c_str(), savefile.c_str());
        } catch (boost::filesystem::filesystem_error& e) {
            std::cout << e.what() << '\n';
        }
        
    }

}