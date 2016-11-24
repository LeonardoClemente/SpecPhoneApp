# SpecPhoneApp
Matlab implementation to help in the image processing for image spectroPhotometers (SpecPhone). 
an image analysis application was developed using MATLAB software. 



Authors : Leonardo Clemente (clemclem1991@gmail.com), Arturo Cevallos (cev.arturo@gmail.com), Luis garza (luis.garza.seriously@gmail.com)

The principal focus of the SpecPhone app is centered in the calculation of intensity and absorbance profiles for each of the samples taken, but it can be also used for reaction kinetics analysis.

Explanation of the processing order is as follows:

1.- Directory browser : Images get imported into an specified folder and prepared for analysis. 
2.- Analysis preparation: Pictures for either the reference and the sample to be analyzed are displayed. The X slider and edit text panel are used to specify the pixel column to analyze. The Y edit and graph length parameters specify the pixel start and finish numbers from the chosen column.
3.- Processing: Intensity profile comparation between the sample and the reference and their absorbance measure based on equation 2. All graphs are normalized to make comparison easier between the other spectrometers. Y label are arbitrary units and X label is wavelength. Wavelength calculation is done prior to  the analysis and implemented internally. In this case, wavelength range is the full visible spectrum (400-700). Wavelength increments are based on picture resolution. It is important to identify both spectrum limits on the image to obtain a coherent mapping. 
4.- Annotations : Posterior to analysis, user can arbitrarily choose which measurements are good and redo those which weren't good enough. Values such as intensity and absorbance profiles, average absorbance and sample concentration are saved. Concentration and average absorbance are displayed within the app for quick check-up.
5.- Regression / Stacking : After experiment is done, the regression button performs simple processing for an easier comparing visualisation.
)
