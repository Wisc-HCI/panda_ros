import numpy as np
import scipy.optimize, scipy.misc



def hessian(x,the_func,epsilon=1e-8):
    """Numerical approximation to the Hessian
    Parameters
    ------------
    x: array-like
        The evaluation point
    the_func: function
        The function. We assume that the function returns the function
value and
        the associated gradient as the second return element
    epsilon: float
        The size of the step
    """

    N = x.size
    h = np.zeros((N,N))
    df_0 = scipy.optimize.approx_fprime(x,the_func,epsilon)
    for i in xrange(N):
        xx0 = 1.*x[i]
        x[i] = xx0 + epsilon
        df_1 = scipy.optimize.approx_fprime(x,the_func,epsilon)
        h[i,:] = (df_1 - df_0)/epsilon
        x[i] = xx0
    return h


def two_stage_fit(first_func,second_func,init_cond,hess = None, second_stage_order = 1):
    first_optim_result = scipy.optimize.minimize(first_func,init_cond)
    if hess is None:
        h = hessian(first_optim_result.x,first_func)
    else:
        h = hess(first_optim_result.x)
    U, s, _ = np.linalg.svd(h)
    print U
    print s
    dirs_inds = np.argsort(s) # find directions that change the objective the least, i.e. the objective is flat
    dirs_inds = np.array([dirs_inds[ii] for ii in range(second_stage_order)]).astype(int)

    dirs = U[:,dirs_inds] # directions to optimize over for second optimization

    transformX_old = lambda x_new : np.matmul(dirs,x_new.reshape(second_stage_order,1)) \
                                    + first_optim_result.x.reshape(len(init_cond),1)
    # total_obj_trans = lambda X:  first_func(transformX_old(X)) + second_func(transformX_old(X))
    total_obj_trans = lambda X:  second_func(transformX_old(X))

    second_optim_result = scipy.optimize.minimize(total_obj_trans, np.random.rand(second_stage_order))
    return transformX_old(second_optim_result.x).reshape(len(init_cond)),first_optim_result.x

