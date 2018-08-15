import numpy as np
import scipy as sp
from scipy import linalg
import numbers
def check_random_state(seed):
    """Turn seed into a np.random.RandomState instance
    If seed is None, return the RandomState singleton used by np.random.
    If seed is an int, return a new RandomState instance seeded with seed.
    If seed is already a RandomState instance, return it.
    Otherwise raise ValueError.
    """
    if seed is None or seed is np.random:
        return np.random.mtrand._rand
    if isinstance(seed, (numbers.Integral, np.integer)):
        return np.random.RandomState(seed)
    if isinstance(seed, np.random.RandomState):
        return seed
    raise ValueError('%r cannot be used to seed a numpy.random.RandomState'
                     ' instance' % seed)


def pinvh(a, cond=None, rcond=None, lower=True):
    """Compute the (Moore-Penrose) pseudo-inverse of a hermetian matrix.
    Calculate a generalized inverse of a symmetric matrix using its
    eigenvalue decomposition and including all 'large' eigenvalues.
    Parameters
    ----------
    a : array, shape (N, N)
        Real symmetric or complex hermetian matrix to be pseudo-inverted
    cond, rcond : float or None
        Cutoff for 'small' eigenvalues.
        Singular values smaller than rcond * largest_eigenvalue are considered
        zero.
        If None or -1, suitable machine precision is used.
    lower : boolean
        Whether the pertinent array data is taken from the lower or upper
        triangle of a. (Default: lower)
    Returns
    -------
    B : array, shape (N, N)
    Raises
    ------
    LinAlgError
        If eigenvalue does not converge
    Examples
    --------
    >>> import numpy as np
    >>> a = np.random.randn(9, 6)
    >>> a = np.dot(a, a.T)
    >>> B = pinvh(a)
    >>> np.allclose(a, np.dot(a, np.dot(B, a)))
    True
    >>> np.allclose(B, np.dot(B, np.dot(a, B)))
    True
    """
    a = np.asarray_chkfinite(a)
    s, u = linalg.eigh(a, lower=lower)

    if rcond is not None:
        cond = rcond
    if cond in [None, -1]:
        t = u.dtype.char.lower()
        factor = {'f': 1E3, 'd': 1E6}
        cond = factor[t] * np.finfo(t).eps

    # unlike svd case, eigh can lead to negative eigenvalues
    above_cutoff = (abs(s) > cond * np.max(abs(s)))
    psigma_diag = np.zeros_like(s)
    psigma_diag[above_cutoff] = 1.0 / s[above_cutoff]
    return np.dot(u * psigma_diag, np.conjugate(u).T)


def invert_indices(n_features, indices):
    inv = np.ones(n_features, dtype=np.bool)
    inv[indices] = False
    inv, = np.where(inv)
    return inv


class MVN(object):
    """Multivariate normal distribution.
    Some utility functions for MVNs. See
    http://en.wikipedia.org/wiki/Multivariate_normal_distribution
    for more details.
    Parameters
    ----------
    mean : array, shape (n_features), optional
        Mean of the MVN.
    covariance : array, shape (n_features, n_features), optional
        Covariance of the MVN.
    verbose : int, optional (default: 0)
        Verbosity level.
    random_state : int or RandomState, optional (default: global random state)
        If an integer is given, it fixes the seed. Defaults to the global numpy
        random number generator.
    """
    def __init__(self, mean=None, covariance=None, verbose=0,
                 random_state=None):
        self.mean = mean
        self.covariance = covariance
        self.verbose = verbose
        self.random_state = check_random_state(random_state)
        self.norm = None

    def _check_initialized(self):
        if self.mean is None:
            raise ValueError("Mean has not been initialized")
        if self.covariance is None:
            raise ValueError("Covariance has not been initialized")

    def from_samples(self, X, bessels_correction=True):
        """MLE of the mean and covariance.
        Parameters
        ----------
        X : array-like, shape (n_samples, n_features)
            Samples from the true function.
        Returns
        -------
        self : MVN
            This object.
        """
        self.mean = np.mean(X, axis=0)
        bias = 0 if bessels_correction else 1
        self.covariance = np.cov(X, rowvar=0, bias=bias)
        self.norm = None
        return self

    def sample(self, n_samples):
        """Sample from multivariate normal distribution.
        Parameters
        ----------
        n_samples : int
            Number of samples.
        Returns
        -------
        X : array, shape (n_samples, n_features)
            Samples from the MVN.
        """
        self._check_initialized()
        return self.random_state.multivariate_normal(
            self.mean, self.covariance, size=(n_samples,))

    def to_probability_density(self, X):
        """Compute probability density.
        Parameters
        ----------
        X : array-like, shape (n_samples, n_features)
            Data.
        Returns
        -------
        p : array, shape (n_samples,)
            Probability densities of data.
        """
        self._check_initialized()

        X = np.atleast_2d(X)
        n_features = X.shape[1]

        C = self.covariance
        try:
            L = sp.linalg.cholesky(C, lower=True)
        except np.linalg.LinAlgError:
            C = self.covariance + 1e-3 * np.eye(n_features)
            L = sp.linalg.cholesky(C, lower=True)
        D = X - self.mean
        cov_sol = sp.linalg.solve_triangular(L, D.T, lower=True).T
        if self.norm is None:
            self.norm = 0.5 / np.pi ** (0.5 * n_features) / sp.linalg.det(L)

        DpD = np.sum(cov_sol ** 2, axis=1)
        return self.norm * np.exp(-0.5 * DpD)

    def marginalize(self, indices):
        """Marginalize over everything except the given indices.
        Parameters
        ----------
        indices : array, shape (n_new_features,)
            Indices of dimensions that we want to keep.
        Returns
        -------
        marginal : MVN
            Marginal MVN distribution.
        """
        self._check_initialized()
        return MVN(mean=self.mean[indices],
                   covariance=self.covariance[np.ix_(indices, indices)])

    def condition(self, indices, x):
        """Conditional distribution over given indices.
        Parameters
        ----------
        indices : array, shape (n_new_features,)
            Indices of dimensions that we want to condition.
        x : array, shape (n_new_features,)
            Values of the features that we know.
        Returns
        -------
        conditional : MVN
            Conditional MVN distribution p(Y | X=x).
        """
        self._check_initialized()
        mean, covariance = self._condition(
            invert_indices(self.mean.shape[0], indices), indices, x)
        return MVN(mean=mean, covariance=covariance,
                   random_state=self.random_state)

    def predict(self, indices, X):
        """Predict means and covariance of posteriors.
        Same as condition() but for multiple samples.
        Parameters
        ----------
        indices : array, shape (n_features_1,)
            Indices of dimensions that we want to condition.
        X : array, shape (n_samples, n_features_1)
            Values of the features that we know.
        Returns
        -------
        Y : array, shape (n_samples, n_features_2)
            Predicted means of missing values.
        covariance : array, shape (n_features_2, n_features_2)
            Covariance of the predicted features.
        """
        self._check_initialized()
        return self._condition(invert_indices(self.mean.shape[0], indices),
                               indices, X)

    def _condition(self, i1, i2, X):
        cov_12 = self.covariance[np.ix_(i1, i2)]
        cov_11 = self.covariance[np.ix_(i1, i1)]
        cov_22 = self.covariance[np.ix_(i2, i2)]
        prec_22 = pinvh(cov_22)
        regression_coeffs = cov_12.dot(prec_22)

        mean = self.mean[i1] + regression_coeffs.dot((X - self.mean[i2]).T).T
        covariance = cov_11 - regression_coeffs.dot(cov_12.T)
        return mean, covariance
class GMM(object):
    def __init__(self, means, covariances, weights):
        self.means_ = means
        self.weights_ = weights
        self.covariances_ = covariances
        self.n_components = self.means_.shape[0]
        self.random_state = None
        self.covariance_type = 'full'
class GMR(object):
    def __init__(self, GMM, priors=None, means=None, covariances=None):
        self.gmm = GMM
        self.priors = priors
        self.means = means
        self.covariances = covariances
    def predict(self, indices, X):
        n_samples, n_features_1 = X.shape
        n_features_2 = self.gmm.means_.shape[1] - n_features_1
        Y = np.empty((n_samples, n_features_2))
        for n in range(n_samples):
            conditioned = self.condition(indices, X[n])
            Y[n] = conditioned.priors.dot(conditioned.means)
        return Y
    def condition(self, indices, x):
        n_features = self.gmm.means_.shape[1] - len(indices)
        priors = np.empty(self.gmm.n_components)
        means = np.empty((self.gmm.n_components, n_features))
        covariances = np.empty((self.gmm.n_components, n_features, n_features))
        for k in range(self.gmm.n_components):
            this_cov = None
            if self.gmm.covariance_type == 'full':
                this_cov = self.gmm.covariances_[k]
            elif self.gmm.covariance_type == 'diag':
                this_cov = np.diag(self.gmm.covariances_[k])
            else:
                print 'wrong'
            mvn = MVN(mean=self.gmm.means_[k], covariance=this_cov,
                      random_state=self.gmm.random_state)
            conditioned = mvn.condition(indices, x)
            priors[k] = (self.gmm.weights_[k] *
                         mvn.marginalize(indices).to_probability_density(x))
            means[k] = conditioned.mean
            covariances[k] = conditioned.covariance
        priors /= priors.sum()
        return GMR(self.gmm, priors=priors, means=means, covariances=covariances)
