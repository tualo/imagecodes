#include "FindCodes.h"

boost::format quicksvfmt("call quicksv('%s','%s','%s','%s','%s', '%s','%s','%s','%s','%s') ");

boost::format set_camera_images_fmt("insert into camera_images (id,inserttime,kunde,state) values ('%s',now(),'%s','%s');  ");
boost::format set_camera_imagescodes_fmt("insert into camera_imagescodes (id,code) values ('%s','%s') on duplicate key update id=values(id)");


FindCodes::FindCodes():
  startSubtractMean(25),
  stepSubtractMean(5),
  
  startBlocksize(45),
  stepBlocksize(10)
{
  maxTasks = boost::thread::hardware_concurrency();
}

FindCodes::~FindCodes() {

}

void FindCodes::detect(std::string filename){

    const char* fname = "";
    fname= filename.c_str();
    cv::Mat img = imread(fname, cv::IMREAD_COLOR );
    
    cv::Mat gray;
    cv::Mat useimage;
    cv::cvtColor(img, useimage, CV_BGR2GRAY);

    detectCodes(useimage);

    int i_bc_thres_stop=85;
    int i_bc_thres_start=5;
    int i_bc_thres_step = 10;

    int subtractMean = 45;
    int blockSize = 55;
}


void FindCodes::detectCodes(cv::Mat image){
    std::list<boost::thread*> threadList;
    for (int bs=startBlocksize;((bs>=0));bs-=stepBlocksize){
        for (int sm=startSubtractMean;((sm>=0));sm-=stepSubtractMean){
            boost::thread* meanLoopThread = new boost::thread(&FindCodes::detectByAdaptiveThreshold, this, image, bs, sm);  
            threadList.push_back(meanLoopThread);
        }
    }
    
    std::list<boost::thread*>::const_iterator it;
    for (it = threadList.begin(); it != threadList.end(); ++it){
        ((boost::thread*)*it)->join();
    }

    //debugCodes();
}
void FindCodes::debugCodes(){
    std::list<Barcode*>::const_iterator it;
    for (it = barcodes.begin(); it != barcodes.end(); ++it){
        ((Barcode*)*it)->printDebug();
    }
}

bool FindCodes::hasCode(std::string code, std::string type){
    std::list<Barcode*>::const_iterator it;
    for (it = barcodes.begin(); it != barcodes.end(); ++it){
        if (
         ((Barcode*)*it)->code()==code
        ){
            if (
            ((Barcode*)*it)->type()==type
            ){
                return true;
            }
        }
        
    }
    return false;
}

std::list<Barcode*> FindCodes::codes(){
    return barcodes;
}


void FindCodes::detectByAdaptiveThreshold(cv::Mat image,int blocksize, int subtractmean) {
        /*
        mutex.lock();
        std::cout << "blocksize: " << blocksize << " " << " subtractmean " << subtractmean << std::endl;
        mutex.unlock();
        */
        cv::Mat gray;
        mutex.lock();
        cv::Mat useimage = image.clone();
        mutex.unlock();

        cv::adaptiveThreshold(
            useimage,
            gray,
            255,
            CV_ADAPTIVE_THRESH_GAUSSIAN_C,
            CV_THRESH_BINARY,
            blocksize,
            subtractmean
        );
        zbar::Image* _image;
        zbar::ImageScanner* _imageScanner;

        _image = new zbar::Image(gray.cols, gray.rows, "Y800", nullptr, 0);
        _imageScanner = new zbar::ImageScanner();
        _imageScanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        _image->set_data((uchar *)gray.data, gray.cols * gray.rows);
        int n = _imageScanner->scan(*_image);
        for(zbar::Image::SymbolIterator symbol = _image->symbol_begin(); symbol != _image->symbol_end(); ++symbol) {

            int n = symbol->get_location_size();
            int xmin = 999999;
            int xmax = 0;
            int ymin = 999999;
            int ymax = 0;
            if(n <= 0){
            //    return(null);
            }else{
                cv::Rect* bounds = new cv::Rect();

                for(int i = 0; i < n; i++) {
                    int x = symbol->get_location_x( i);
                    if(xmin > x) xmin = x;
                    if(xmax < x) xmax = x;

                    int y = symbol->get_location_y( i);
                    if(ymin > y) ymin = y;
                    if(ymax < y) ymax = y;
                }
                bounds->x = xmin;
                bounds->y = ymin;
                bounds->width = xmax - xmin;
                bounds->height = ymax - ymin;
                mutex.lock();
                if (hasCode(symbol->get_data().c_str(), symbol->get_type_name().c_str())){

                }else{

                    barcodes.push_back( new Barcode(
                        symbol->get_data().c_str(),
                        symbol->get_type_name().c_str(),
                        bounds->x ,
                        bounds->y ,
                        bounds->width ,
                        bounds->height 
                    ) );
                }


                
                //std::cout << "bounds->x: " << bounds->x << " " << "bounds->y: " << bounds->y << " bounds->width " << bounds->width << " bounds->height " << bounds->height << std::endl;
                mutex.unlock();
            }

        


        }
        _image->set_data(NULL, 0);
}
