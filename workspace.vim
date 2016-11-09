set path +=.,~/dobot/**

nnoremap <silent> <Leader>mm :wa<cr> :make -C ~/dobot/<cr> :cw<cr>
nnoremap <Leader>mc :make clean -C ~/dobot/<cr>
nnoremap <Leader>mi :make install -C ~/dobot/<cr>
