
#include <jni.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>
extern "C"
{

        JNIEXPORT JNICALL
            jlong
            Java_org_firstinspires_ftc_teamcode_junction_LUT_createPtrToLut(JNIEnv *env,
                                                                            __attribute__((unused)) jclass lutObject, jbyteArray lut)
        {
                auto *rawLut = new jbyte[256 * 256 * 256];
                jboolean boolean = JNI_FALSE;
                jbyte *lutElements = env->GetByteArrayElements(lut, &boolean);
                memcpy(rawLut, lutElements, sizeof(jbyte) * 256 * 256 * 256);
                env->ReleaseByteArrayElements(lut, lutElements, 0);
                return (jlong)rawLut;
        }
        JNIEXPORT JNICALL void Java_org_firstinspires_ftc_teamcode_junction_LUT_lutOperation(JNIEnv *env, jclass lutObject, jlong inputPtr, jlong outputPtr)
        {
                static uint8_t *lut = nullptr;
                auto *inputMat = (cv::Mat *)inputPtr;
                auto *outputMat = (cv::Mat *)outputPtr;

                assert(inputMat->isContinuous() == true);
                assert(inputMat->type() == CV_8UC3);
                assert(outputMat->isContinuous() == true);

                if (__builtin_expect(lut == nullptr, 0))
                {
                        lut = (uint8_t *)env->GetStaticLongField(lutObject, env->GetStaticFieldID(lutObject, "ptrToLut", "J"));
                }
                const auto numPixels = inputMat->cols * inputMat->rows;

                for (auto i = 0; i < numPixels; i++)
                {
                        const auto offset = inputMat->data + i * 3;
                        const auto b = *(offset);
                        const auto g = *(offset + 1);
                        const auto r = *(offset + 2);

                        outputMat->data[i] = (lut[r * 256 * 256 + g * 256 + b]>0)*255;
                }
        }
}