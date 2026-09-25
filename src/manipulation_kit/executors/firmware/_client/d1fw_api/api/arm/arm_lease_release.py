from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_lease_release import ArmLeaseRelease
from ...models.arm_lease_release_response_200 import ArmLeaseReleaseResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ArmLeaseRelease,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "delete",
        "url": "/v1/arm/lease",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmLeaseReleaseResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmLeaseReleaseResponse200.from_dict(response.json())

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
) -> Response[ArmLeaseReleaseResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRelease,
) -> Response[ArmLeaseReleaseResponse200 | ErrorEnvelope]:
    """Hand the arms back

     Releases only if `holder` matches the current holder; releasing somebody else's lease is refused
    with 409. Releasing when no lease is held succeeds, so a consumer shutting down need not know
    whether its own lease already expired.

    Args:
        body (ArmLeaseRelease): `DELETE /v1/arm/lease`: hand the arms back.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmLeaseReleaseResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRelease,
) -> ArmLeaseReleaseResponse200 | ErrorEnvelope | None:
    """Hand the arms back

     Releases only if `holder` matches the current holder; releasing somebody else's lease is refused
    with 409. Releasing when no lease is held succeeds, so a consumer shutting down need not know
    whether its own lease already expired.

    Args:
        body (ArmLeaseRelease): `DELETE /v1/arm/lease`: hand the arms back.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmLeaseReleaseResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRelease,
) -> Response[ArmLeaseReleaseResponse200 | ErrorEnvelope]:
    """Hand the arms back

     Releases only if `holder` matches the current holder; releasing somebody else's lease is refused
    with 409. Releasing when no lease is held succeeds, so a consumer shutting down need not know
    whether its own lease already expired.

    Args:
        body (ArmLeaseRelease): `DELETE /v1/arm/lease`: hand the arms back.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmLeaseReleaseResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRelease,
) -> ArmLeaseReleaseResponse200 | ErrorEnvelope | None:
    """Hand the arms back

     Releases only if `holder` matches the current holder; releasing somebody else's lease is refused
    with 409. Releasing when no lease is held succeeds, so a consumer shutting down need not know
    whether its own lease already expired.

    Args:
        body (ArmLeaseRelease): `DELETE /v1/arm/lease`: hand the arms back.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmLeaseReleaseResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
