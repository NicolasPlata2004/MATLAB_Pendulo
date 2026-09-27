function exportarFiguraVectorial(figura,carpeta,nombreBase)
%EXPORTARFIGURAVECTORIAL Guarda una figura en PDF, EPS y PNG.

% Crear la carpeta si todavía no existe.
if ~isfolder(carpeta)
    mkdir(carpeta);
end

archivoPDF = fullfile(carpeta,nombreBase + ".pdf");
archivoEPS = fullfile(carpeta,nombreBase + ".eps");
archivoPNG = fullfile(carpeta,nombreBase + ".png");

% PDF vectorial.
exportgraphics(figura,archivoPDF,'ContentType','vector');

% EPS vectorial.
print(figura,char(archivoEPS),'-depsc','-painters');

% PNG de alta resolución.
exportgraphics(figura,archivoPNG,'Resolution',300);

end