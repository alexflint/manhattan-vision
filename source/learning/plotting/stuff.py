


#MultiViewDepthLoss = 'experiments/feb21_labellingloss_allfeatures_4offsets'
MultiViewDepthLoss = 'experiments/feb21_depthloss_allfeatures_4offsets'
#MultiViewLabellingLoss = 'experiments/feb21_depthloss_allfeatures_4offsets'

#MultiViewLabellingLoss = 'experiments/mar07_labellingloss_multiview-all_large'
MultiViewLabellingLoss = 'experiments/feb21_labellingloss_allfeatures_4offsets'
#MultiViewDepthLoss = 'experiments/feb21_labellingloss_allfeatures_4offsets'

#SingleViewDepthLoss = 'experiments/mar05_depthloss_mono-all_large'
SingleViewDepthLoss = 'experiments/mar05_depthloss_mono-all_large_iter472man'

SingleViewLabellingLoss = 'experiments/mar05_labellingloss_mono-all_large'

ICCV = 'experiments/feb22_iccvparams'

ECCV = 'experiments/mar05_eccv2010'

Experiments = {
    'mview_depth': MultiViewDepthLoss,
    'mview_lbl':   MultiViewLabellingLoss,
    'sview_depth': SingleViewDepthLoss,
    'sview_lbl':   SingleViewLabellingLoss,
    'iccv':        ICCV,
    'eccv':        ECCV,
    }
