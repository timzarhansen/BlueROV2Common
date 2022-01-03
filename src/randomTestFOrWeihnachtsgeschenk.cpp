#include "vector"
#include <iostream>

std::vector<int> addTwoVectors(std::vector<int> first , std::vector<int> second){
    for(int i = 0;i<first.size();i++){
        first[i]=first[i]+second[i];
    }
    return first;

}

std::vector<int> rotateRightByNumberOfPositions(std::vector<int> array0 , int numberOfTimes){

    std::rotate(array0.begin(),
                array0.end()-numberOfTimes, // this will be the new first element
                array0.end());
    return array0;
}


std::vector<int> removeEverySecond(std::vector<int> vectorInput,bool beginRemoveFirst){

    if(beginRemoveFirst){
        for(int i = 0;i<vectorInput.size();i+=2){
            vectorInput[i]=0;
        }
    }else{
        for(int i = 1;i<vectorInput.size();i+=2){
            vectorInput[i]=0;
        }
    }
    return vectorInput;
}

std::vector<int> zeroArrayReturn(int index1){
    std::vector<int> array0{19,10,13,10,2,15,23,19,3,2,3,27,20,11,27,10};
    std::vector<int> array1{12,24,10,9,22,9,5,10,5,1,24,2,10,9,7,3};
    bool beginFirst = false;
    if(index1 % 2 == 0){
        beginFirst = true;
    }
    std::vector<int> returnArray = addTwoVectors(array0,removeEverySecond(array1,beginFirst));
    return returnArray;
}

std::vector<int> oneArrayReturn(int index1,int index2){
    bool beginFirst = false;
    if(index2 % 2 == 0){
        beginFirst = true;
    }
    std::vector<int> array0{6,0,9,0,16,0,17,0,2,0,2,0,10,0,15,0};
    std::vector<int> array1{22,6,1,1,11,27,14,5,5,7,8,24,8,3,6,15};
    array0 = rotateRightByNumberOfPositions(array0,index1);
    array1 = rotateRightByNumberOfPositions(array1,index1);
    array1 = removeEverySecond(array1,beginFirst);
    std::vector<int> returnArray = addTwoVectors(array0,array1);
    return returnArray;
}

std::vector<int> twoArrayReturn(int index2,int index3){
    bool beginFirst = false;
    if(index3 % 2 == 0){
        beginFirst = true;
    }
    std::vector<int> array0{2,0,22,0,2,0,17,0,15,0,14,0,5,0,10,0};
    std::vector<int> array1{10,6,10,2,6,10,4,1,5,5,4,8,6,3,1,6};
    array0 = rotateRightByNumberOfPositions(array0,index2);
    array1 = rotateRightByNumberOfPositions(array1,index2);
    array1 = removeEverySecond(array1,beginFirst);
    std::vector<int> returnArray = addTwoVectors(array0,array1);
    return returnArray;

}

std::vector<int> threeArrayReturn(int index3){

    std::vector<int> array0{13,0,3,0,3,0,6,0,10,0,10,0,10,0,6,0};
    array0 = rotateRightByNumberOfPositions(array0,index3);
    return array0;
}

int main(int argc, char **argv) {
    int index1 = 0;
    int index2 = 0;
    int index3 = 0;
    for( index1 = 0;index1<16;index1++) {
        for (index2 = 0; index2 < 16; index2++) {
            for (index3 = 0; index3 < 16; index3++) {


                std::vector<int> zeroReturn = zeroArrayReturn(index1);
                std::vector<int> firstReturn = oneArrayReturn(index1, index2);
                std::vector<int> secondReturn = twoArrayReturn(index2, index3);
                std::vector<int> thirdReturn = threeArrayReturn(index3);
                std::vector<int> completeSum = addTwoVectors(zeroReturn, firstReturn);
                completeSum = addTwoVectors(completeSum, secondReturn);
                completeSum = addTwoVectors(completeSum, thirdReturn);

                //    for(int i = 0;i<completeSum.size();i+=1) {
                //        std::cout << completeSum[i] << std::endl;
                //    }

                if (completeSum[0] == completeSum[1] && completeSum[1] == completeSum[2] && completeSum[3] == completeSum[4] && completeSum[6] == completeSum[9]) {
                    for (int i = 0; i < completeSum.size(); i += 1) {
                        std::cout << completeSum[i] << std::endl;
                    }
                    std::cout << "indexes: " << index1 << index2 << index3 << std::endl;
                }
            }
        }
    }
    return (0);
}


