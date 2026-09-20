from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_recover_request import ArmRecoverRequest
from ...models.arm_recover_response_200 import ArmRecoverResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...types import UNSET, Response, Unset


def _get_kwargs(
    side: ArmSide,
    *,
    body: ArmRecoverRequest | Unset = UNSET,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/recover".format(
            side=quote(str(side), safe=""),
        ),
    }

    if not isinstance(body, Unset):
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmRecoverResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmRecoverResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ArmRecoverResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmRecoverRequest | Unset = UNSET,
) -> Response[ArmRecoverResponse200 | ErrorEnvelope]:
    """Recover one arm to a measured-pose position hold

     Clears errors, anchors the commanded pose at the measured pose and confirms position mode. This
    commands motion. `vel_ratio` and `acc_ratio` are fractions of full speed in `0.0..=1.0`, defaulting
    to `0.05`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmRecoverRequest | Unset): Explicit recovery command; ratios are normalized, not
            percentages.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmRecoverResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmRecoverRequest | Unset = UNSET,
) -> ArmRecoverResponse200 | ErrorEnvelope | None:
    """Recover one arm to a measured-pose position hold

     Clears errors, anchors the commanded pose at the measured pose and confirms position mode. This
    commands motion. `vel_ratio` and `acc_ratio` are fractions of full speed in `0.0..=1.0`, defaulting
    to `0.05`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmRecoverRequest | Unset): Explicit recovery command; ratios are normalized, not
            percentages.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmRecoverResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        side=side,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmRecoverRequest | Unset = UNSET,
) -> Response[ArmRecoverResponse200 | ErrorEnvelope]:
    """Recover one arm to a measured-pose position hold

     Clears errors, anchors the commanded pose at the measured pose and confirms position mode. This
    commands motion. `vel_ratio` and `acc_ratio` are fractions of full speed in `0.0..=1.0`, defaulting
    to `0.05`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmRecoverRequest | Unset): Explicit recovery command; ratios are normalized, not
            percentages.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmRecoverResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmRecoverRequest | Unset = UNSET,
) -> ArmRecoverResponse200 | ErrorEnvelope | None:
    """Recover one arm to a measured-pose position hold

     Clears errors, anchors the commanded pose at the measured pose and confirms position mode. This
    commands motion. `vel_ratio` and `acc_ratio` are fractions of full speed in `0.0..=1.0`, defaulting
    to `0.05`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmRecoverRequest | Unset): Explicit recovery command; ratios are normalized, not
            percentages.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmRecoverResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
