# STM32_SCREEN_LTDC_DCMI_OV7670
Bare metal project for a fully functional digital camera using STM32. This is the last project in a sequence of projects.

## General description
This is the concluding project for reproducing the Adafruit SAMD51 digital camera solution using an STM32 discovery board instead. It uses mostly code that has been produced by me beforehand (and has been shared as a repository) with little to no modification.

Nevertheless, I think it is good to touch upon a few things and maybe reflect on the projects that have been done here.

## Previous relevant projects
We will build on the previous project in the line:
- STM32_SCREEN_LTDC_DCMI_OV7670

Get the push button working from here:
- STM32_NVMDriver

And port the following project to work with STM32:
- SAMD21_SPI_SD_fatfs

## To read
I suggest reading the Wikipedia article on the BMP file format. 

## Particularities

### Porting to F429
As mentioned above, we are re-using the SDcard and the fatfs code that have been generated for the SAMD21 microcontroller. Even though that project was written using a C++, the code is directly compatible with a C environment with practically no changes needed to it. Same can be said for the blue push button from the related STM32 project.

One note here: I did run into some rather annoying issues when porting the SPI, though these were related to the GPIO pins of the F429 disco board. As it goes, the GPIO pins PB14 and PB15 (SPI2 MISO and SPI2 MOSI, respectively) are “broken out” on the disco board, but they aren’t connected to their pins unless the SB26 and SB27 pads are soldered up. I am absolutely flabbergasted by why on earth did ST do this and it take me a few days to figure out, why otherwise perfectly functional code doesn’t seem to work on these pins.

Lastly, be advised that the code that is presented originally in the SAMD21 project assumes that we are using an SDHC capable SDcard in the external card reader. This means that all 32 GB should work fine, but anything less would run into issues (sector definition problem). More of this was discussed in the “SAMD21_SPI_SD_fatfs” project. Either way, if one wishes to use some alternative type/size card, the card initiation should be adjusted (not discussed here).

### BMP generation
The new element in this project is the generation of an image file.

Image file type we will be using is the “BMP” (or bitmap) type. The reason for that is simple: it is very easy to generate it. As it goes, the way how a file system defines a file type is by opening it up and then checking the first sequence of bytes (called the header) and not by what the extension of the file is. As such, if we know the header we need to generate an image and then write this header it into a properly defined/sized memory buffer, that memory buffer will be understood by a file system looking at that memory buffer as an “image file”. This is obviously very crucial information when transitioning from a bare metal approach to an operating system. BMP file headers are 70 bytes (if we don’t want to go fancy).

BMP files are not used much in computing though for the simple reason that they are – usually – not compressed. This means that they comparatively take up significantly more memory than their PNG or JPEG counterparts. On the brighter side, exactly because they are uncompressed, logging data into them is completely straight forwards.

Technically, what we need to do in a bare metal environment to “build” a BMP file is just open a memory section (“f_open” command in the fat_fs library) and then write to the file the BMP header, followed by the 16-bit RGB565 image data. Once the adequate number of pixels are transferred into the memory buffer, the file can be closed with “f_close” and that’s it.

## User guide
All the features from the previous project are still available, should it be necessary.

Regarding how to use the setup, one merely needs to push the blue push button on the disco board to capture the image shown on the screen. The result will be a 240x240 resolution BMP file.

Of small note, the image number counter only goes until 255 in the current version. Once that number is reached, the camera will not take any more images until the SDcard is emptied.

## Conclusion
I just wanted to do this small addition to properly reproduce what the guys at Adafruit have managed to do using their SAMD51 but using an STM32F429 instead. Of course, we are cheating a bit compared to them since we are generating 240x240 images instead of 240x320, but the speed increase justifies that decision. We also don’t have resolution flexibility and other fancy control stuff that makes their project so juicy…but all those things could be added to my solution rather easily now that the backbone of the project is finished.

At any rate, this project was fun and challenging to do and I am glad I did it. We will see if I come up with a follow-up or just do something completely different instead.
