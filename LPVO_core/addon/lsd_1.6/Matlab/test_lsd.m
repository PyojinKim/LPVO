X=double(imread('input.png'));
% The image has to be between 0 and 255.
% The image has to be black and white

output_lsd=lsd(X);
figure,
hold on
imagesc(X);
colormap gray;
for i=1:size(output_lsd,1)
    plot([output_lsd(i,1),output_lsd(i,3)],[output_lsd(i,2),output_lsd(i,4)]);
end
axis ij
hold off
